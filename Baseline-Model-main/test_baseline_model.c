/* Host-side harness: compiles baseline_model.h and replays every dataset
 * row through the REAL compiled C, comparing against Python's prediction.
 *
 * build & run:
 *   gcc -O2 -std=c99 -I outputs test_baseline_model.c -o /tmp/test_bl
 *   /tmp/test_bl outputs/_vectors.csv
 */
#include <stdio.h>
#include <stdlib.h>
#include "baseline_model.h"

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s vectors.csv\n", argv[0]); return 2; }
    FILE *fp = fopen(argv[1], "r");
    if (!fp) { perror("open"); return 2; }

    char line[2048];
    long n = 0, ok = 0;
    int first_bad = -1;

    while (fgets(line, sizeof line, fp)) {
        float x[BL_N_FEATURES];
        int expected;
        char *p = line;
        for (int i = 0; i < BL_N_FEATURES; i++) {
            x[i] = strtof(p, &p);
            if (*p == ',') p++;
        }
        expected = (int)strtof(p, &p);

        uint8_t got = baseline_predict(x);
        if (got == expected) ok++;
        else if (first_bad < 0) first_bad = (int)n;
        n++;
    }
    fclose(fp);

    printf("rows replayed through compiled C : %ld\n", n);
    printf("matching Python model            : %ld  (%.4f%%)\n",
           ok, 100.0 * (double)ok / (double)n);
    if (ok != n) {
        printf("FAIL — first mismatch at row %d.\n", first_bad);
        return 1;
    }
    printf("PASS — compiled baseline logic == Python model.\n");
    return 0;
}
