/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Dayane
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
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
//  Header
// ============================================================================

#include "ufr.h"
#include "ufr_test.h"
#include "assert.h"

// ============================================================================
//  Tests
// ============================================================================

void test1() {
    printf ("Iniciando testes...\n");

    ufr_buffer_t buffer;
    ufr_buffer_init(&buffer); // Inicializa o buffer
    UFR_TEST_EQUAL_U64( buffer.size, 0L ); /* Verifica se o tamanho inicial do buffer é 0 */ 
    UFR_TEST_EQUAL_U64( buffer.max, MESSAGE_ITEM_SIZE ); /* Verifica se a capacidade máxima do
                                                          * buffer é igual a MESSAGE_ITEM_SIZE 
                                                          * (um valor definido em algum lugar do código). */
    ufr_buffer_put(&buffer, "opa", 3); /* Adiciona a string "opa" (3 bytes) ao buffer*/
    UFR_TEST_EQUAL_U64( buffer.size, 3L ); /* Verifica o tamanho do buffer */
    ufr_buffer_clear(&buffer);
    UFR_TEST_EQUAL_U64(buffer.size, 0);
}

void test2() {
    ufr_buffer_t* buffer = ufr_buffer_new(); /* Cria um novo buffer dinamicamente*/
    UFR_TEST_NOT_NULL(buffer); /* Verifica se o ponteiro retornado não é NULL
                                  * usando a macro UFR_TEST_NOT_NULL*/
    ufr_buffer_free (buffer);                             
}

void teste_equal () {
    unsigned long valor = 5;

    ufr_buffer_t* buffer = ufr_buffer_new();
    UFR_TEST_NOT_NULL(buffer);

    ufr_buffer_free(buffer);
    UFR_TEST_NULL(buffer->ptr);
    /*if (buffer->ptr == NULL){
        printf ("NAO LIBEROU MEMORIA");

    }*/
    UFR_TEST_EQUAL_U64(buffer->max, 0L);
    UFR_TEST_EQUAL_U64(buffer->size, 0L);

    //UFR_TEST_EQUAL (valor, 5);
    // UFR_TEST_EQUAL (valor, 10);
    //UFR_TEST_TRUE (valor);
}

int main() {
    test1();
    printf ("Teste 1 feito!\n");
    test2();
    printf ("Teste 2 feito!\n");
    teste_equal();
   // printf ("Teste equal feito!\n");

    ufr_test_print_result();
    return 0;
}
