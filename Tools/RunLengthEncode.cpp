#include "raw_image.h"
#include <stdio.h>

int RunLengthEncode(unsigned char arr[], int arr_length, FILE* fp);

// Run-Length encodes the images in arrays "image" and "logo" in "raw_image.H"
// Outputs to file "image.h"
// Copy image.h file over to the Libraries directory to change logo.
int main(){

    FILE *fp = fopen("image.h", "w");
    fprintf(fp, "unsigned char image[] = {");
    int new_arr_length = 0;
    int arr_length = sizeof(image) / sizeof(image[0]);
    new_arr_length = RunLengthEncode(image, arr_length, fp);
    float reduction = 100 * (1 - ((float) new_arr_length / arr_length));
    printf("Image compressed from %d bytes to %d bytes (%.2f%% reduction)\n", arr_length, new_arr_length, reduction);

    fprintf(fp, "\n\nunsigned char logo[] = {");

    arr_length = sizeof(logo) / sizeof(logo[0]);
    new_arr_length = RunLengthEncode(logo, arr_length, fp);
    reduction = 100 * (1 - ((float) new_arr_length / arr_length));
    printf("Logo compressed from %d bytes to %d bytes (%.2f%% reduction)\n", arr_length, new_arr_length, reduction);

}


int RunLengthEncode(unsigned char arr[], int arr_length, FILE* fp){
    int len = 1;
    unsigned char character = arr[0];
    int i = 0;
    int compressed_length = 0;

    for(i = 1; i < arr_length; i++){
        // keep track of number of same characters in a row (up to 255 since this is stored to a char array)
        if(character == arr[i] && len < 255){
            len++;
        }else{
        // if we found a mismatched character, print out the run length, then character
            fprintf(fp, "%d, %c, ", len, character + '0');
            compressed_length += 2;
            // update char, reset run length
            character = arr[i];
            len = 1;
        }
    }
    fprintf(fp, "%d, %c};", len, character + '0');
    return compressed_length + 2;
}
