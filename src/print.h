#ifndef SRC_PRINT_H
#define SRC_PRINT_H

#include <stdio.h>
#include "ev3_light.h"

/** Display the error and activate the led.
 *
 */
void displayError(const char* message)
{
    printf("Error : %s\n", message);
    while(true)
    {
        switch ( get_light( LIT_LEFT ))
        {
            case LIT_GREEN:
                set_light( LIT_LEFT, LIT_RED );
                break;
            case LIT_RED:
                set_light( LIT_LEFT, LIT_AMBER );
                break;
            default:
                set_light( LIT_LEFT, LIT_GREEN );
                break;
        }
    }
}

#endif //SRC_PRINT_H
