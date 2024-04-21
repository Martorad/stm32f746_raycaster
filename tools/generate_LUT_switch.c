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
    sprintf(fileName, "LUT_%li.h", time(NULL));

    FILE* f = fopen(fileName, "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        return 1;
    }

    fprintf(f, "// TANGENTS\n");
    fprintf(f, "float getTangent(uint32_t n) {\n");
    fprintf(f, "  switch (n) {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "case %i: return %.012f; break; ", i, tan(i / 1000.0 + 0.000001));
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n  default: return 0; break;");
    fprintf(f, "\n  }\n}\n");

    fprintf(f, "// SINES\n");
    fprintf(f, "float getSine(uint32_t n) {\n");
    fprintf(f, "  switch (n) {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "case %i: return %.012f; break; ", i, sin(i / 1000.0 + 0.000001));
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n  default: return 0; break;");
    fprintf(f, "\n  }\n}\n");

    fprintf(f, "// COSINES\n");
    fprintf(f, "float getCosine(uint32_t n) {\n");
    fprintf(f, "  switch (n) {\n  ");
    for (uint16_t i = 0; i < resolution; i++) { 
        fprintf(f, "case %i: return %.012f; break; ", i, cos(i / 1000.0 + 0.000001));
        if (i % 10 == 9) { fprintf(f, "\n  "); }
    }
    fprintf(f, "\n  default: return 0; break;");
    fprintf(f, "\n  }\n}\n");

    fclose(f);
    return 0;
}