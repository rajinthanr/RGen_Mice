#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C"
{
#endif

    extern bool is_key_pressed;
    extern bool is_boot_pressed;

    void button_Configuration(void);

#ifdef __cplusplus
}
#endif

#endif
