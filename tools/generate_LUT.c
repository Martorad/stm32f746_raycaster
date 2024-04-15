#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

int main(int argc, char* argv[]) {
    if (argc == 1) { 
        printf("Please provide a resolution.\n");
        return 1;
    } 

    uint32_t resolution = strtol(argv[1], NULL, 10);
    char fileName[64];
    sprintf(fileName, "LUT_%li.txt", time(NULL));

    FILE* f = fopen(fileName, "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        return 1;
    }

    fprintf(f, "// TANGENTS\n");
    fprintf(f, "const float _tanLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "%.012f", tan(i / 1000.0 + 0.000001));
        if (i != resolution - 1) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n\n");

    fprintf(f, "// SINES\n");
    fprintf(f, "const float _sinLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "%.012f", sin(i / 1000.0 + 0.000001));
        if (i != resolution - 1) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n\n");

    fprintf(f, "// COSINES\n");
    fprintf(f, "const float _cosLUT[M_LUT_S] = {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "%.012f", cos(i / 1000.0 + 0.000001));
        if (i != resolution - 1) { fprintf(f, ", "); } 
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n};\n");

    fclose(f);
    return 0;
}