#include <assert.h>
#include <stdio.h>
#include <dlfcn.h>
#include <stdint.h>
#include <string.h>
#include <ufr.h>

typedef uint8_t (*dl_func_version_t)(uint8_t index);
typedef const char* (*dl_func_list_t)(uint8_t index);
typedef int(*dl_func_new_t)(link_t*, const lt_args_t* args);


int test_publisher() {

}

int main() {
    link_t link;
    // assert( ufr_sys_load_library(&link, "ufr_gtw_posix") == LT_OK );

    for (int i=0; i<10; i++) {
        const char* name = ufr_sys_lib_call_list(&link, i);
        if ( name == NULL ) {
            break;
        }
        printf("%s\n", name);
    }

    // assert( ufr_sys_lib_call_new(&link, "file", NULL) == LT_OK );

    lt_args_t args = {.text="@path saida.txt"};
    ufr_boot(&link, &args);
    ufr_start(&link, &args);
    lt_write(&link, "opa\n", 4);
    lt_close(&link);
    return 0;
}