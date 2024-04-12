#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

int main() {
    FILE* f = fopen("./tangent_values.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        return 1;
    }

    fprintf(f, "{\n");
    for (uint16_t i = 0; i < 6284; i++) { 
        fprintf(f, "%.012f", tan(i / 1000.0 + 0.000001));
        if (i != 6283) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n"); }
    }
    fprintf(f, "\n}");

    fclose(f);
    return 0;
}