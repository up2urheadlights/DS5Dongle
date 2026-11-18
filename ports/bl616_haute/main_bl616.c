/* Bouffalo SDK entry point for the BL616 Haute target.
 *
 * The SDK builds its "main file" into the executable itself, outside the app
 * library that carries the firmware's include paths and definitions, so this
 * file only hands over to entry_bl616.cpp. */
extern int ds5_bl616_entry(void);

int main(void)
{
    return ds5_bl616_entry();
}
