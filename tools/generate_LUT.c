#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

int main() {
    FILE* f = fopen("./LUT.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        return 1;
    }

    fprintf(f, "// TANGENTS\n");
    fprintf(f, "const float _tanLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < 6284; i++) { 
        fprintf(f, "%.012f", tan(i / 1000.0 + 0.000001));
        if (i != 6283) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n\n");

    fprintf(f, "// SINES\n");
    fprintf(f, "const float _sinLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < 6284; i++) { 
        fprintf(f, "%.012f", sin(i / 1000.0 + 0.000001));
        if (i != 6283) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n\n");

    fprintf(f, "// COSINES\n");
    fprintf(f, "const float _cosLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < 6284; i++) { 
        fprintf(f, "%.012f", cos(i / 1000.0 + 0.000001));
        if (i != 6283) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n");

    fclose(f);
    return 0;
}