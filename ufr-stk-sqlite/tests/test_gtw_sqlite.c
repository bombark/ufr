/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
//  HEADER
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ufr.h>

int ufr_gtw_sqlite_new(link_t* link, const ufr_args_t* args);

// ============================================================================
//  Tests
// ============================================================================
/*
void test_select() {
    link_t link;
    ufr_args_t args_boot = {.text="@file test.db"};
    ufr_new_gtw_sqlite_table(&link, &args_boot);

    ufr_args_t args_subs = {.text="@sql %s", .arg[0].str="SELECT * FROM pessoa"};
    ufr_start_subscriber(&link, &args_subs);

    int id;
    char name[64];
    int score;

    ufr_get(&link, "^is", &id, name);
    printf("%d %s\n", id, name);

    ufr_get(&link, "^is", &id, name);
    printf("%d %s\n", id, name);

    ufr_get(&link, "^is", &id, name);
    printf("%d %s\n", id, name);

    ufr_close(&link);
}

void test_insert() {
    link_t link;
    ufr_args_t args_boot = {.text="@file test.db"};
    ufr_new_gtw_sqlite_table(&link, &args_boot);

    ufr_args_t args_subs = {.text="@sql %s", .arg[0].str="INSERT INTO pessoa VALUES(?,?,?)"};
    ufr_start_publisher(&link, &args_subs);

    ufr_put(&link, "isi\n", 7, "trrr", 80);
    ufr_put(&link, "isi\n", 4, "teste", 50);
    ufr_put(&link, "isi\n", 5, "teste", 60);
    ufr_put(&link, "isi\n", 6, "teste", 70);


    ufr_close(&link);
}
*/

// ============================================================================
//  Main
// ============================================================================

int main() {
    // test_select();
    // test_insert();
	return 0;
}