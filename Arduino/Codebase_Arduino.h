/*
 * Some constants for displaying colours
 */
#define DIM_MODE 0

/*
 * A simple method which compares two colour arrays for equality
 */
boolean matchingColours(uint8_t *colourArrayOne, uint8_t *colourArrayTwo, int arraySize);

/*
 * Below are declarations for the different colour patterns
 */

/*
 * A simple pattern which has all lights display the given colour
 */
void blockPattern();

/*
 * A pattern which upon successive calls will change alternate a colour
 * between light and dark. 
 */
void pulsePattern();

/*
 * Generates a complimentary colour and displays it in alternating positions
 */
void complimentaryPattern();
