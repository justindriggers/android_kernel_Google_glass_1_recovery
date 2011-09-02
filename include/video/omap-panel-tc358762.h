#ifndef __VIDEO_TC358762_BOARD_DATA_H__
#define __VIDEO_TC358762_BOARD_DATA_H__

struct tc358762_board_data {
        int reset_gpio;
        void (*pre_reset)(void);
        void (*post_reset)(void);
};

#endif
