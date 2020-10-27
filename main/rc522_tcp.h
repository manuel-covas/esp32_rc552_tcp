#ifndef RC552_TCP_H
#define RC552_TCP_H


typedef enum {
    SPI_TRANSCEIVE,
    RST_GPIO_GET,
    RST_GPIO_SET
}
rc522_tcp_operation_t;

typedef enum {
    OPERATION_OK,
    OPERATION_FAIL
}
rc522_tcp_operation_success_t;

typedef enum {
    GPIO_LOW,
    GPIO_HIGH
}
rc522_tcp_gpio_state_t;


#endif