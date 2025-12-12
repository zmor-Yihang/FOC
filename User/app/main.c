#include "main.h"

void clock_config(void);

int main(void)
{
    HAL_Init();
    clock_config();
    
    while (1)
    {
    }
}


