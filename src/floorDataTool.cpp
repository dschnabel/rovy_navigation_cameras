// compile with: g++ floorDataTool.cpp -o floorDataTool

#include <fstream>
#include <string.h>

#define FLOOR_MATRIX_WIDTH   640
#define FLOOR_MATRIX_HEIGHT   145

struct floorDataPair {
    int min;
    int max;
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("no argument given\n");
        return -1;
    }

    bool min = false;
    bool merge = false;

    if (strcmp(argv[1], "min") == 0) {
        min = true;
    } else if (strcmp(argv[1], "max") == 0) {
        min = false;
    } else if (strcmp(argv[1], "merge") == 0) {
        merge = true;
    } else {
        printf("wrong argument, must be one of: min, max, merge\n");
        return -1;
    }

    std::ifstream floorDataFile("floorData.bin", std::ios::binary);

    if (floorDataFile.good()) {

        if (merge) {
            std::ifstream floorDataFile2("floorData2.bin", std::ios::binary);
            std::ofstream floorDataFileMerged("floorDataMerged.bin", std::ios::binary);
            if (!floorDataFile2.good()) {
                printf("file error: floorData2.bin\n");
                return -1;
            }
            if (!floorDataFileMerged.good()) {
                printf("file error: floorDataMerged.bin\n");
                return -1;
            }
            for (int x = 0; x < FLOOR_MATRIX_WIDTH; x++) {
                for (int y = 0; y < FLOOR_MATRIX_HEIGHT; y++) {
                    struct floorDataPair pair1, pair2, pairMerged;
                    floorDataFile.read((char*)&pair1, sizeof(pair1));
                    floorDataFile2.read((char*)&pair2, sizeof(pair2));

                    pairMerged.min = pair1.min < pair2.min ? pair1.min : pair2.min;
                    pairMerged.max = pair1.max > pair2.max ? pair1.max : pair2.max;
                    floorDataFileMerged.write((char*)&pairMerged, sizeof(pairMerged));
                }
            }
            floorDataFileMerged.close();
            floorDataFile2.close();
            floorDataFile.close();

            return 0;
        }

        if (min) {
            printf("Minimum\n=======\n");
        } else {
            printf("Maximum\n=======\n");
        }
        for (int y = 0; y < FLOOR_MATRIX_HEIGHT; y++) {
            printf(",%d", y);
        }
        printf("\n");

        for (int x = 0; x < FLOOR_MATRIX_WIDTH; x++) {
            printf("%d", x);
            for (int y = 0; y < FLOOR_MATRIX_HEIGHT; y++) {
                struct floorDataPair pair;
                floorDataFile.read((char*)&pair, sizeof(pair));
                if (min) {
                    printf(",%d", pair.min);
                } else {
                    printf(",%d", pair.max);
                }
            }
            printf("\n");
        }
        floorDataFile.close();
    } else {
        printf("file error: floorData.bin\n");
    }
   return 0;
}
