#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "sensors.h"
#include "modbus_rtu.h"

int main(void)
{
    printk("Modbus master demarre\n");

    while (1) {
        for (int i = 0; i < NB_CAPTEURS; i++) {
            int16_t val;

            if (modbus_read_register(capteurs[i].adresse,
                                     capteurs[i].reg_temp,
                                     &val)) {
                printk("[%s] Temp: %d.%02d C\n",
                    capteurs[i].nom,
                    val / capteurs[i].facteur_temp,
                    val % capteurs[i].facteur_temp);
            }

            k_msleep(50);
        }

        k_sleep(K_SECONDS(60));
    }
}