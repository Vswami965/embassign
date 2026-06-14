#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define SOF             0xAAU
#define MAX_PAYLOAD     16U

#define FRAME_OK         1
#define FRAME_IN_PROGRESS 0
#define CHECKSUM_ERROR  -1
#define TIMEOUT_ERROR   -2

typedef enum
{
    STATE_WAIT_SOF = 0,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM
} ParserState;

typedef struct
{
    ParserState state;

    uint8_t cmd;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];

    uint8_t checksum;
    uint8_t payload_index;

    uint32_t timeout_ms;
    uint32_t last_timestamp;

} UARTParser;

static void parser_reset(UARTParser *parser)
{
    parser->state = STATE_WAIT_SOF;
    parser->cmd = 0U;
    parser->len = 0U;
    parser->checksum = 0U;
    parser->payload_index = 0U;
}

static void parser_init(UARTParser *parser, uint32_t timeout_ms)
{
    memset(parser, 0, sizeof(UARTParser));

    parser->state = STATE_WAIT_SOF;
    parser->timeout_ms = timeout_ms;
}

static int parser_feed_byte(UARTParser *parser,
                            uint8_t byte,
                            uint32_t timestamp)
{
    if ((parser->timeout_ms > 0U) &&
        (parser->state != STATE_WAIT_SOF))
    {
        uint32_t gap = timestamp - parser->last_timestamp;

        if (gap > parser->timeout_ms)
        {
            parser_reset(parser);
            return TIMEOUT_ERROR;
        }
    }

    parser->last_timestamp = timestamp;

    switch (parser->state)
    {
        case STATE_WAIT_SOF:

            if (byte == SOF)
            {
                parser->checksum = 0U;
                parser->payload_index = 0U;
                parser->state = STATE_WAIT_CMD;
            }
            break;

        case STATE_WAIT_CMD:

            parser->cmd = byte;
            parser->checksum ^= byte;
            parser->state = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:

            parser->len = byte;
            parser->checksum ^= byte;

            if (parser->len > MAX_PAYLOAD)
            {
                parser_reset(parser);
                break;
            }

            if (parser->len == 0U)
            {
                parser->state = STATE_WAIT_CHECKSUM;
            }
            else
            {
                parser->state = STATE_WAIT_PAYLOAD;
            }
            break;

        case STATE_WAIT_PAYLOAD:

            parser->payload[parser->payload_index] = byte;
            parser->payload_index++;
            parser->checksum ^= byte;

            if (parser->payload_index >= parser->len)
            {
                parser->state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM:

            if (byte == parser->checksum)
            {
                return FRAME_OK;
            }
            else
            {
                parser_reset(parser);
                return CHECKSUM_ERROR;
            }

        default:
            parser_reset(parser);
            break;
    }

    return FRAME_IN_PROGRESS;
}

static void print_frame(const UARTParser *parser)
{
    uint8_t i;

    printf("FRAME OK CMD=0x%02X LEN=%u PAYLOAD=[",
           parser->cmd,
           parser->len);

    for (i = 0U; i < parser->len; i++)
    {
        printf("%02X", parser->payload[i]);

        if (i < (parser->len - 1U))
        {
            printf(" ");
        }
    }

    printf("]\n");
}

static void feed_stream(UARTParser *parser,
                        const uint8_t *bytes,
                        const uint32_t *times,
                        uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        int result;

        result = parser_feed_byte(parser,
                                  bytes[i],
                                  times[i]);

        printf("t=%ums byte=0x%02X -> ",
               times[i],
               bytes[i]);

        if (result == FRAME_IN_PROGRESS)
        {
            printf("receiving...\n");
        }
        else if (result == FRAME_OK)
        {
            print_frame(parser);
            parser_reset(parser);
        }
        else if (result == CHECKSUM_ERROR)
        {
            printf("CHECKSUM ERROR\n");
        }
        else if (result == TIMEOUT_ERROR)
        {
            printf("TIMEOUT - parser reset\n");

            result = parser_feed_byte(parser,
                                      bytes[i],
                                      times[i]);

            if (result == FRAME_IN_PROGRESS)
            {
                printf("t=%ums byte=0x%02X -> receiving...\n",
                       times[i],
                       bytes[i]);
            }
        }
    }
}

int main(void)
{
    UARTParser parser;

    /* Test 1 */
    uint8_t test1[] =
    {
        0xAA, 0x01, 0x03,
        0x10, 0x20, 0x30,
        0x22
    };

    uint32_t time1[] =
    {
        0, 5, 10, 15, 20, 25, 30
    };

    printf("\n===== TEST 1 =====\n");

    parser_init(&parser, 50U);

    feed_stream(&parser,
                test1,
                time1,
                (uint32_t)(sizeof(test1) / sizeof(test1[0])));

    /* Test 2 */

    uint8_t test2[] =
    {
        0xAA, 0x01, 0x03, 0x10,
        0xAA, 0x05, 0x01, 0x7F, 0x7B
    };

    uint32_t time2[] =
    {
        0, 5, 10, 15,
        200, 205, 210, 215, 220
    };

    printf("\n===== TEST 2 =====\n");

    parser_init(&parser, 50U);

    feed_stream(&parser,
                test2,
                time2,
                (uint32_t)(sizeof(test2) / sizeof(test2[0])));

    /* Test 3 */

    uint8_t test3[] =
    {
        0xAA, 0x03, 0x01, 0x55, 0x57,
        0xAA, 0x04, 0x02, 0xAA, 0xBB, 0x15
    };

    uint32_t time3[] =
    {
        0, 5, 10, 15, 20,
        25, 30, 35, 40, 45, 50
    };

    printf("\n===== TEST 3 =====\n");

    parser_init(&parser, 50U);

    feed_stream(&parser,
                test3,
                time3,
                (uint32_t)(sizeof(test3) / sizeof(test3[0])));

    /* Test 4 */

    printf("\n===== TEST 4 (TIMEOUT DISABLED) =====\n");

    parser_init(&parser, 0U);

    feed_stream(&parser,
                test2,
                time2,
                (uint32_t)(sizeof(test2) / sizeof(test2[0])));

    return 0;
}