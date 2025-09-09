# Programação em Assembly para STM32F411 Black Pill: Um Guia Detalhado

Este documento serve como um guia abrangente para entender e programar microcontroladores STM32 em linguagem Assembly, utilizando como base o projeto de exemplo de piscar um LED na placa STM32F411 Black Pill. A programação em Assembly, embora mais complexa que linguagens de alto nível como C/C++, oferece controle granular sobre o hardware, otimização de desempenho e compreensão aprofundada da arquitetura do sistema.

## 1. Introdução à Linguagem Assembly em Sistemas Embarcados

### O que é Assembly?

A linguagem Assembly é uma linguagem de programação de baixo nível que possui uma correspondência quase direta com as instruções de máquina que o processador executa. Cada instrução em Assembly geralmente corresponde a uma única instrução de máquina. Ao invés de usar códigos binários (0s e 1s), o Assembly utiliza mnemônicos (abreviações) para representar as operações, tornando o código mais legível para humanos do que o código de máquina puro.

### Por que usar Assembly para Microcontroladores?

Em sistemas embarcados, onde recursos como memória e poder de processamento são frequentemente limitados, o Assembly pode ser uma ferramenta poderosa por várias razões:

*   **Controle Direto do Hardware**: O Assembly permite manipular diretamente os registradores do microcontrolador e os periféricos, oferecendo um controle preciso sobre o hardware que não é facilmente alcançável com linguagens de alto nível.
*   **Otimização de Desempenho**: Para tarefas críticas de tempo ou rotinas que exigem máxima velocidade, o Assembly permite escrever código altamente otimizado, superando as otimizações de compiladores em alguns casos específicos.
*   **Tamanho do Código**: O código Assembly pode ser extremamente compacto, o que é crucial para microcontroladores com memória Flash limitada.
*   **Depuração de Baixo Nível**: Entender Assembly é fundamental para depurar problemas complexos em sistemas embarcados, especialmente quando se lida com interrupções, drivers de hardware ou falhas de sistema.
*   **Compreensão da Arquitetura**: Programar em Assembly força o desenvolvedor a entender profundamente a arquitetura do processador, o conjunto de instruções, o mapa de memória e o funcionamento dos periféricos.

### Vantagens e Desvantagens

| Vantagens                                   | Desvantagens                               |
| :------------------------------------------ | :----------------------------------------- |
| Controle granular do hardware               | Curva de aprendizado íngreme               |
| Otimização de desempenho e tamanho do código | Código menos legível e mais propenso a erros |
| Acesso direto a registradores e periféricos | Dificuldade na depuração                   |
| Essencial para drivers e bootloaders        | Portabilidade limitada entre arquiteturas  |
| Compreensão aprofundada da arquitetura      | Maior tempo de desenvolvimento             |

### Arquitetura ARM Cortex-M (Cortex-M4)

O STM32F411 Black Pill utiliza um processador ARM Cortex-M4. Esta é uma arquitetura RISC (Reduced Instruction Set Computer) de 32 bits, otimizada para microcontroladores. Algumas características importantes incluem:

*   **Conjunto de Instruções Thumb-2**: Uma mistura de instruções de 16 e 32 bits que oferece boa densidade de código e desempenho.
*   **Registradores**: O Cortex-M4 possui 16 registradores de uso geral (R0-R12), um Stack Pointer (SP, R13), um Link Register (LR, R14) e um Program Counter (PC, R15). Além disso, possui registradores especiais como o Program Status Register (PSR).
*   **Memória Mapeada**: Periféricos e memória são acessados através de endereços de memória específicos. Isso simplifica a interação com o hardware, pois a leitura e escrita em registradores de periféricos são feitas como operações de memória comuns.
*   **Pipeline**: O Cortex-M4 utiliza um pipeline de 3 estágios para execução de instruções, o que melhora o desempenho.
*   **FPU (Floating Point Unit)**: O Cortex-M4 possui uma unidade de ponto flutuante opcional, que acelera operações matemáticas com números de ponto flutuante.

## 2. Entendendo a Arquitetura do STM32F411 Black Pill

A placa STM32F411 Black Pill é uma placa de desenvolvimento compacta baseada no microcontrolador STM32F411CEU6 da STMicroelectronics. Para programar em Assembly, é crucial entender como seus componentes internos estão organizados e como interagir com eles.

### Componentes Chave

*   **CPU**: ARM Cortex-M4 com FPU, operando a até 100 MHz.
*   **Memória Flash**: 512 KB para armazenamento de código do programa.
*   **SRAM (RAM)**: 128 KB para armazenamento de dados e pilha.
*   **Periféricos**: Uma vasta gama de periféricos, incluindo GPIOs (General Purpose Input/Output), RCC (Reset and Clock Control), Timers, UART, SPI, I2C, ADC, etc.

### Registradores Mapeados na Memória

No STM32, a interação com os periféricos é feita através de registradores mapeados na memória. Cada periférico possui um conjunto de registradores que controlam seu comportamento e estado. Para manipular um periférico, você simplesmente lê ou escreve nos endereços de memória correspondentes a esses registradores.

Por exemplo, para controlar os pinos GPIO, usamos registradores como:

*   **MODER (Mode Register)**: Define o modo de operação de cada pino (entrada, saída, função alternativa, analógico).
*   **OTYPER (Output Type Register)**: Define o tipo de saída (push-pull ou open-drain).
*   **OSPEEDR (Output Speed Register)**: Define a velocidade de saída do pino.
*   **PUPDR (Pull-up/Pull-down Register)**: Configura resistores de pull-up ou pull-down.
*   **IDR (Input Data Register)**: Lê o estado dos pinos configurados como entrada.
*   **ODR (Output Data Register)**: Escreve o estado dos pinos configurados como saída.
*   **BSRR (Bit Set/Reset Register)**: Permite setar (colocar em alto) ou resetar (colocar em baixo) bits específicos dos pinos de saída de forma atômica.

### O LED On-board (PC13)

Na maioria das placas STM32F411 Black Pill, há um LED on-board conectado ao pino **PC13**. É importante notar que este LED é geralmente **ativo baixo**, o que significa que ele acende quando o pino PC13 é configurado para um nível lógico **baixo (0V)** e apaga quando configurado para um nível lógico **alto (3.3V)**.

Para controlar o PC13, precisamos interagir com os registradores do **GPIOC** (General Purpose Input/Output Port C). Os endereços base dos registradores para GPIOC e RCC (que controla o clock) são:

*   **GPIOC Base Address**: `0x40020800`
*   **RCC Base Address**: `0x40023800`

Com base nesses endereços, podemos derivar os endereços específicos dos registradores que usaremos:

*   **RCC_AHB1ENR**: `0x40023830` (Endereço base RCC + offset 0x30)
*   **GPIOC_MODER**: `0x40020800` (Endereço base GPIOC + offset 0x00)
*   **GPIOC_BSRR**: `0x40020818` (Endereço base GPIOC + offset 0x18)

Para mais detalhes sobre os registradores e seus offsets, consulte o **Manual de Referência do STM32F411** (disponível no site da STMicroelectronics). Este documento é a fonte definitiva para informações sobre os periféricos do microcontrolador.

## 3. Dissecando o Código Assembly de Exemplo (`src/main.s`)

O código `src/main.s` é um programa simples que faz o LED on-board (PC13) piscar. Vamos analisar cada seção em detalhes.

```assembly
@ Arquivo: main.s
@ Descrição: Este código em Assembly faz o LED on-board (conectado ao pino PC13) do STM32F411 Black Pill piscar.
@            Ele demonstra a configuração básica de GPIO e um loop de atraso simples.
@ Arquitetura: ARM Cortex-M4 (Thumb-2)

.syntax unified
.cpu cortex-m4
.thumb

.global _start

_start:
    @ Ponto de entrada principal do programa.
    @ A execução começa aqui após o reset do microcontrolador.
    BL enable_gpio_clock    @ Chama a sub-rotina para habilitar o clock da porta GPIO C.
    BL setup_gpio           @ Chama a sub-rotina para configurar o pino PC13 como saída.

loop:
    @ Loop principal do programa. O LED piscará continuamente dentro deste loop.
    BL turn_on_led          @ Chama a sub-rotina para ligar o LED.
    BL delay                @ Chama a sub-rotina para criar um atraso (LED aceso).
    BL turn_off_led         @ Chama a sub-rotina para desligar o LED.
    BL delay                @ Chama a sub-rotina para criar um atraso (LED apagado).
    B loop                  @ Salta incondicionalmente de volta para o início do 'loop', criando um ciclo infinito.

enable_gpio_clock:
    @ Sub-rotina para habilitar o clock da porta GPIOC.
    @ O LED on-board do STM32F411 Black Pill está conectado ao pino PC13.
    @ Para que qualquer periférico funcione, seu clock deve ser habilitado no RCC (Reset and Clock Control).
    LDR r0, =0x40023830     @ Carrega o endereço base do registrador RCC_AHB1ENR (AHB1 Peripheral Clock Enable Register) em R0.
                            @ Este registrador controla o clock dos periféricos conectados ao barramento AHB1.
    LDR r1, [r0]            @ Lê o valor atual do registrador RCC_AHB1ENR para R1.
    LDR r3, =0x04           @ Carrega o valor 0x04 (binário 0000_0100) em R3.
                            @ O bit 2 (0x04) corresponde ao enable do clock para GPIOC (Bit 2: GPIOCEN).
    ORR r1, r1, r3          @ Realiza uma operação OR bit a bit entre R1 (valor atual do registrador) e R3 (máscara para GPIOCEN).
                            @ Isso garante que o bit GPIOCEN seja setado para 1, sem alterar outros bits.
    STR r1, [r0]            @ Armazena o novo valor (com o clock do GPIOC habilitado) de volta no registrador RCC_AHB1ENR.
    BX lr                   @ Retorna da sub-rotina para o endereço de retorno armazenado em LR (Link Register).

setup_gpio:
    @ Sub-rotina para configurar o pino PC13 como saída de propósito geral (General Purpose Output).
    @ O modo de operação dos pinos GPIO é configurado através do registrador MODER (Mode Register).
    LDR r0, =0x40020800     @ Carrega o endereço base do registrador GPIOC_MODER (GPIO Port Mode Register) em R0.
                            @ Para o pino PC13, os bits 26 e 27 controlam o modo (MODER13[1:0]).
    LDR r1, [r0]            @ Lê o valor atual do registrador GPIOC_MODER para R1.
    LDR r3, =0xC000000      @ Carrega o valor 0xC000000 (binário 1100_0000_0000_0000_0000_0000_0000_0000) em R3.
                            @ Esta máscara é usada para limpar os bits 26 e 27 (MODER13[1:0]), garantindo que eles estejam zerados antes de definir o novo modo.
    BIC r1, r1, r3          @ Realiza uma operação BIC (Bit Clear) entre R1 e R3.
                            @ Isso limpa os bits 26 e 27 de R1, preparando-os para a nova configuração.
    LDR r3, =0x4000000      @ Carrega o valor 0x4000000 (binário 0100_0000_0000_0000_0000_0000_0000_0000) em R3.
                            @ Este valor define o modo de saída de propósito geral (01b) para os bits 27:26.
    ORR r1, r1, r3          @ Realiza uma operação OR bit a bit entre R1 e R3.
                            @ Isso define os bits 26 e 27 para '01', configurando PC13 como saída.
    STR r1, [r0]            @ Armazena o novo valor (com PC13 configurado como saída) de volta no registrador GPIOC_MODER.
    BX lr                   @ Retorna da sub-rotina.

turn_on_led:
    @ Sub-rotina para ligar o LED conectado ao pino PC13.
    @ O LED na Black Pill é ativo baixo, o que significa que ele acende quando o pino é configurado para nível lógico baixo (0V).
    @ Para isso, usamos o bit BR13 (Bit Reset) no registrador BSRR (Bit Set/Reset Register).
    LDR r0, =0x40020818     @ Carrega o endereço base do registrador GPIOC_BSRR (GPIO Port Bit Set/Reset Register) em R0.
                            @ Este registrador permite setar ou resetar bits específicos dos pinos GPIO.
    LDR r1, =0x20000000     @ Carrega o valor 0x20000000 (binário 0010_0000_0000_0000_0000_0000_0000_0000) em R1.
                            @ Este valor corresponde ao bit 29 (BR13), que quando escrito como 1, reseta o pino PC13 (coloca em nível baixo).
    STR r1, [r0]            @ Armazena o valor em R1 no registrador GPIOC_BSRR, ligando o LED.
    BX lr                   @ Retorna da sub-rotina.

turn_off_led:
    @ Sub-rotina para desligar o LED conectado ao pino PC13.
    @ Como o LED é ativo baixo, para desligá-lo, precisamos configurar o pino para nível lógico alto (3.3V).
    @ Para isso, usamos o bit BS13 (Bit Set) no registrador BSRR.
    LDR r0, =0x40020818     @ Carrega o endereço base do registrador GPIOC_BSRR em R0.
    LDR r1, =0x00002000     @ Carrega o valor 0x00002000 (binário 0000_0000_0000_0000_0010_0000_0000_0000) em R1.
                            @ Este valor corresponde ao bit 13 (BS13), que quando escrito como 1, seta o pino PC13 (coloca em nível alto).
    STR r1, [r0]            @ Armazena o valor em R1 no registrador GPIOC_BSRR, desligando o LED.
    BX lr                   @ Retorna da sub-rotina.

delay:
    @ Sub-rotina de atraso simples baseada em um loop de decremento.
    @ Este é um atraso de software e sua duração exata depende da frequência do clock da CPU.
    LDR r2, =500000         @ Carrega o valor imediato 500000 em R2. Este valor determina a duração do atraso.
                            @ Um valor maior resulta em um atraso mais longo.

delay_loop:
    SUB r2, r2, #1          @ Decrementa o valor em R2 por 1.
    CMP r2, #0              @ Compara o valor de R2 com 0.
    BNE delay_loop          @ Se R2 não for igual a 0 (Branch if Not Equal), salta de volta para 'delay_loop'.
                            @ O loop continua até que R2 chegue a 0.
    BX lr                   @ Retorna da sub-rotina de atraso.
```

### Explicação das Instruções ARM (Thumb-2) Utilizadas

*   **`LDR` (Load Register)**: Carrega um valor da memória para um registrador. Ex: `LDR R0, =0x40023830` carrega o endereço `0x40023830` para R0. `LDR R1, [R0]` carrega o conteúdo do endereço apontado por R0 para R1.
*   **`STR` (Store Register)**: Armazena o valor de um registrador na memória. Ex: `STR R1, [R0]` armazena o conteúdo de R1 no endereço apontado por R0.
*   **`BL` (Branch with Link)**: Salta para uma sub-rotina e armazena o endereço da próxima instrução no registrador LR (Link Register). Isso permite que a sub-rotina retorne ao ponto de chamada.
*   **`BX LR` (Branch and Exchange Link Register)**: Retorna de uma sub-rotina para o endereço armazenado em LR. O `X` indica que o estado do Thumb pode ser trocado (não relevante aqui, mas é a instrução padrão para retorno).
*   **`B` (Branch)**: Salta incondicionalmente para um rótulo. Ex: `B loop` salta para o rótulo `loop`.
*   **`ORR` (Logical OR)**: Realiza uma operação OR bit a bit entre dois operandos e armazena o resultado no registrador de destino. Usado para setar bits.
*   **`BIC` (Bit Clear)**: Realiza uma operação AND NOT bit a bit. Limpa bits específicos. Ex: `BIC R1, R1, R3` limpa os bits em R1 que estão setados em R3.
*   **`SUB` (Subtract)**: Subtrai um valor de um registrador. Ex: `SUB R2, R2, #1` subtrai 1 de R2.
*   **`CMP` (Compare)**: Compara dois valores e atualiza os flags de status (Zero, Negative, Carry, Overflow) no registrador PSR. O resultado da comparação não é armazenado em um registrador, apenas os flags são afetados.
*   **`BNE` (Branch if Not Equal)**: Salta para um rótulo se o flag Zero (Z) for 0 (ou seja, se a comparação anterior resultou em valores diferentes).

### Fluxo de Execução

1.  **`_start`**: O programa começa aqui. Ele chama as funções de inicialização do GPIO e entra no loop principal.
2.  **`enable_gpio_clock`**: Habilita o clock para a porta GPIOC, que é essencial para que o microcontrolador possa interagir com os pinos dessa porta.
3.  **`setup_gpio`**: Configura o pino PC13 como saída. Isso envolve a manipulação do registrador `GPIOC_MODER` para definir o modo correto.
4.  **`loop`**: O coração do programa. Alterna entre ligar e desligar o LED, com atrasos entre as operações.
5.  **`turn_on_led`**: Liga o LED escrevendo no registrador `GPIOC_BSRR` para resetar o bit correspondente ao PC13 (lembre-se, LED ativo baixo).
6.  **`turn_off_led`**: Desliga o LED escrevendo no registrador `GPIOC_BSRR` para setar o bit correspondente ao PC13.
7.  **`delay`**: Uma rotina de atraso baseada em um loop de decremento. A duração do atraso é proporcional ao valor inicial carregado em R2.

## 4. Conversão entre C/C++ e Assembly

Entender como o código C/C++ é traduzido para Assembly é fundamental para otimização e depuração. O compilador (neste caso, `arm-none-eabi-gcc`) é responsável por essa tradução. Vamos ver alguns exemplos.

### C/C++ para Assembly

Considere a seguinte função simples em C:

```c
void add_numbers(int a, int b, int* result) {
    *result = a + b;
}
```

Um compilador ARM pode traduzir isso para algo semelhante a (simplificado):

```assembly
add_numbers:
    @ a em R0, b em R1, result em R2 (convenção de chamada ARM EABI)
    ADD R0, R0, R1      @ R0 = R0 + R1 (a + b)
    STR R0, [R2]        @ Armazena o resultado (em R0) no endereço apontado por R2 (*result = R0)
    BX LR               @ Retorna da função
```

**Outro exemplo: Loop `for` em C para Assembly**

```c
void simple_loop(int count) {
    for (int i = 0; i < count; i++) {
        // Do something (e.g., toggle a pin)
    }
}
```

Em Assembly (simplificado):

```assembly
simple_loop:
    @ count em R0
    MOV R1, #0          @ R1 = 0 (i = 0)

loop_start:
    CMP R1, R0          @ Compara i com count
    BGE loop_end        @ Se i >= count, sai do loop

    @ ... código para 


    @ ... código para 'Do something'

    ADD R1, R1, #1      @ Incrementa i (i++)
    B loop_start        @ Volta para o início do loop

loop_end:
    BX LR               @ Retorna da função
```

### Assembly para C/C++

É possível chamar funções escritas em Assembly a partir de código C/C++. Isso é comum para rotinas de inicialização de baixo nível, drivers de hardware ou funções críticas de desempenho.

**Exemplo: Chamando uma função Assembly de C**

Suponha que temos a função `delay` em Assembly, como no nosso projeto. Podemos declará-la em C da seguinte forma:

```c
// Em um arquivo .h ou no topo do seu arquivo .c
extern void delay(void);

int main() {
    // ... seu código de inicialização
    while(1) {
        // Ligar LED
        delay();
        // Desligar LED
        delay();
    }
}
```

O compilador C/C++ saberá que `delay` é uma função externa e procurará sua implementação no código Assembly durante a fase de linking. A convenção de chamada (Application Binary Interface - ABI) da ARM EABI (Embedded Application Binary Interface) define como os parâmetros são passados para as funções (geralmente nos registradores R0-R3) e como os valores de retorno são tratados (geralmente em R0).

## 5. Estrutura do Projeto PlatformIO para Assembly

O PlatformIO é uma plataforma de desenvolvimento unificada para sistemas embarcados que simplifica o processo de compilação, upload e depuração. Embora seja mais comumente usado com C/C++, ele pode ser configurado para compilar código Assembly.

### `platformio.ini`

O arquivo `platformio.ini` é o coração da configuração do projeto PlatformIO. Ele define a placa, o framework (se houver), as flags de compilação, o script do linker e outras opções. Para compilar Assembly, algumas configurações são cruciais:

```ini
[env:blackpill_f411ce]
platform = ststm32
board = blackpill_f411ce

; Flags de compilação para Assembly
build_flags =
  -DSTM32F4              ; Define a família do microcontrolador
  -DSTM32F411xE           ; Define o modelo específico do microcontrolador
  -DSTM32F411CEU6         ; Define o part number específico
  -DUSE_HAL_DRIVER        ; Pode ser útil se você misturar com código C/HAL
  -D__ASSEMBLY__          ; Indica ao compilador que estamos lidando com Assembly
  -mthumb                 ; Gera código Thumb-2 (instruções de 16/32 bits)
  -mcpu=cortex-m4         ; Especifica a CPU alvo
  -mfloat-abi=hard        ; Usa hardware FPU para ponto flutuante
  -mfpu=fpv4-sp-d16       ; Especifica a FPU

; Script do linker personalizado para definir o mapa de memória
board_build.ldscript = STM32F411CEUx_FLASH.ld

; Filtro de arquivos fonte para incluir apenas arquivos .s (Assembly)
source_filter = +<src/*.s>

; upload_protocol = stlink ; Descomente para configurar o método de upload
; debug_tool = stlink     ; Descomente para configurar a ferramenta de depuração
```

### `STM32F411CEUx_FLASH.ld` (Linker Script)

O linker script (`.ld`) é um arquivo essencial que informa ao linker como organizar as diferentes seções do seu programa (código, dados, pilha, heap) na memória do microcontrolador. Ele define os endereços de início e os tamanhos da memória Flash e RAM.

```ld
/* linker.ld */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  .text : { *(.text) } > FLASH
  .data : { *(.data) } > RAM
  .bss : { *(.bss) } > RAM
}
```

*   **`MEMORY`**: Define as regiões de memória disponíveis no microcontrolador.
    *   `FLASH (rx)`: Memória Flash, onde o código do programa é armazenado. `rx` indica que é legível e executável.
        *   `ORIGIN = 0x08000000`: Endereço de início da Flash no STM32F411.
        *   `LENGTH = 512K`: Tamanho da memória Flash (512 Kilobytes).
    *   `RAM (rwx)`: Memória RAM (SRAM), usada para variáveis, pilha e heap. `rwx` indica que é legível, gravável e executável.
        *   `ORIGIN = 0x20000000`: Endereço de início da RAM no STM32F411.
        *   `LENGTH = 128K`: Tamanho da memória RAM (128 Kilobytes).
*   **`SECTIONS`**: Define como as seções do seu programa serão mapeadas para as regiões de memória.
    *   `.text`: Contém o código executável. Mapeado para a `FLASH`.
    *   `.data`: Contém variáveis inicializadas. Mapeado para a `RAM`.
    *   `.bss`: Contém variáveis não inicializadas (zeradas pelo sistema). Mapeado para a `RAM`.

### Estrutura de Pastas

Um projeto PlatformIO típico para Assembly terá a seguinte estrutura:

```
. (raiz do projeto)
├── src/
│   └── main.s         ; Seu código Assembly
├── include/
│   └── README         ; Arquivos de cabeçalho (se houver, para C/C++)
├── lib/
│   └── README         ; Bibliotecas personalizadas (se houver)
├── platformio.ini     ; Arquivo de configuração do PlatformIO
├── STM32F411CEUx_FLASH.ld ; Script do linker
└── README.md          ; Este arquivo de documentação
```

## 6. Ferramentas e Ambiente de Desenvolvimento

Para trabalhar com Assembly para STM32, você precisará de algumas ferramentas essenciais:

### GNU ARM Embedded Toolchain

Este é o conjunto de ferramentas padrão para desenvolvimento em ARM. Inclui:

*   **`arm-none-eabi-as` (Assembler)**: Converte o código Assembly (`.s`) em arquivos objeto (`.o`).
*   **`arm-none-eabi-ld` (Linker)**: Combina os arquivos objeto e bibliotecas, usando o linker script, para criar o executável (`.elf`).
*   **`arm-none-eabi-objcopy`**: Converte o executável (`.elf`) para formatos binários (`.bin`) ou hexadecimais (`.hex`) para upload no microcontrolador.
*   **`arm-none-eabi-gdb` (Debugger)**: Ferramenta de depuração.

Você pode baixar o toolchain em: [https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

### PlatformIO

Como demonstrado neste projeto, o PlatformIO abstrai muitas das complexidades da cadeia de ferramentas e da configuração do projeto. Ele gerencia automaticamente as dependências, o toolchain e as configurações da placa. Para instalá-lo, você pode usar `pip` (gerenciador de pacotes Python):

```bash
pip install platformio
```

### OpenOCD (Open On-Chip Debugger)

OpenOCD é uma ferramenta de código aberto que fornece depuração e programação para sistemas embarcados usando interfaces como JTAG e SWD. É essencial para carregar seu código compilado no STM32.

Download: [http://openocd.org/](http://openocd.org/)

### ST-Link V2/V3

Um programador/depurador de hardware necessário para conectar seu STM32 Black Pill ao computador e transferir o firmware. É a interface física entre o OpenOCD e o microcontrolador.

## 7. Como Compilar e Carregar o Projeto

### Usando PlatformIO (Recomendado)

1.  **Clone ou baixe este repositório.**
2.  **Navegue até a pasta raiz do projeto** no seu terminal.
3.  **Execute o comando de compilação do PlatformIO**:
    ```bash
    pio run
    ```
    Isso irá compilar o código Assembly e gerar o arquivo `.elf` e `.bin` na pasta `.pio/build/blackpill_f411ce/`.
4.  **Para carregar o código (flashing)**, você precisará de um programador ST-Link conectado à sua placa Black Pill. Se o PlatformIO estiver configurado corretamente (descomente as linhas `upload_protocol` e `debug_tool` no `platformio.ini`), você pode simplesmente executar:
    ```bash
    pio run --target upload
    ```

### Compilação Manual (Para Entendimento Profundo)

Se você quiser entender o processo de compilação em um nível mais baixo, pode usar as ferramentas GNU ARM Embedded Toolchain diretamente:

1.  **Certifique-se de que o toolchain esteja no seu PATH.**
2.  **Navegue até a pasta `src/`** do projeto.
3.  **Compile o arquivo Assembly para um arquivo objeto**:
    ```bash
    arm-none-eabi-as -mcpu=cortex-m4 -mthumb main.s -o main.o
    ```
4.  **Linke o arquivo objeto com o linker script** (certifique-se de que `STM32F411CEUx_FLASH.ld` esteja no mesmo diretório ou especifique o caminho completo):
    ```bash
    arm-none-eabi-ld -T STM32F411CEUx_FLASH.ld main.o -o firmware.elf
    ```
5.  **Converta o arquivo ELF para BIN** (formato binário puro, mais fácil de carregar):
    ```bash
    arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
    ```

### Carregamento (Flashing) Manual com OpenOCD

1.  **Conecte seu ST-Link** ao STM32 Black Pill (SWDIO, SWCLK, GND, VCC).
2.  **Inicie o OpenOCD** em um terminal (ajuste os arquivos de configuração conforme sua instalação):
    ```bash
    openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
    ```
    *   `interface/stlink.cfg`: Configuração para o seu programador ST-Link.
    *   `target/stm32f4x.cfg`: Configuração para a família STM32F4.
3.  **Abra um segundo terminal e conecte-se ao servidor telnet do OpenOCD**:
    ```bash
    telnet localhost 4444
    ```
4.  **No terminal telnet, execute os comandos para carregar o firmware**:
    ```
    reset halt
    flash write_image erase firmware.bin 0x08000000
    reset run
    exit
    ```

## 8. Referências e Recursos Adicionais

Para aprofundar seus conhecimentos em Assembly ARM e programação de microcontroladores STM32, os seguintes recursos são altamente recomendados:

### Documentação Oficial da ARM e STMicroelectronics

*   **ARM Architecture Reference Manual**: A documentação definitiva para o conjunto de instruções ARM. Essencial para entender cada instrução em detalhes.
    *   [https://developer.arm.com/documentation/ddi0406/latest/](https://developer.arm.com/documentation/ddi0406/latest/)
*   **STM32F411 Reference Manual (RM0383)**: O manual de referência específico para o microcontrolador STM32F411. Contém todos os detalhes sobre os periféricos, registradores e mapa de memória.
    *   Procure por 


    *   Procure por `RM0383` no site da STMicroelectronics: [https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
*   **STM32F411 Datasheet (DS10707)**: Fornece informações elétricas e características gerais do microcontrolador.
    *   Procure por `DS10707` no site da STMicroelectronics.

### Livros e Tutoriais

*   **"The Definitive Guide to ARM Cortex-M3 and Cortex-M4 Processors" por Joseph Yiu**: Um livro excelente e aprofundado sobre a arquitetura Cortex-M, registradores, conjunto de instruções e programação.
*   **"Embedded Systems with ARM Cortex-M Microcontrollers in Assembly Language and C" por Yifeng Zhu**: Um recurso prático para aprender a programar microcontroladores ARM em Assembly e C.
*   **Tutoriais online**: Muitos blogs e sites oferecem tutoriais sobre programação Assembly para ARM e STM32. Pesquise por termos como "ARM Assembly tutorial", "STM32 GPIO Assembly", etc.

### Ferramentas e Comunidades

*   **PlatformIO Documentation**: [https://docs.platformio.org/](https://docs.platformio.org/)
*   **ARM Developer Website**: [https://developer.arm.com/](https://developer.arm.com/)
*   **STMicroelectronics Website**: [https://www.st.com/](https://www.st.com/)
*   **Fóruns e Comunidades**: Participe de fóruns como o EEVblog, Stack Overflow, ou comunidades específicas de STM32 para tirar dúvidas e aprender com outros desenvolvedores.

## 9. Visualizações e Diagramas (SVG)

Para facilitar a compreensão de conceitos complexos, diagramas visuais são extremamente úteis. Embora não seja possível gerar SVGs diretamente aqui, posso descrever exemplos de diagramas que seriam benéficos para este README:

*   **Diagrama de Blocos do STM32F411**: Uma representação visual dos principais componentes do microcontrolador (CPU, Flash, RAM, Periféricos, Barramentos) e suas interconexões. Isso ajuda a entender a arquitetura geral.
*   **Mapa de Memória do STM32F411**: Um diagrama que ilustra as regiões de memória (Flash, RAM, Periféricos) e seus endereços de início e fim. Isso é crucial para entender como o linker script funciona e onde o código e os dados são armazenados.
*   **Diagrama de Fluxo do Programa (LED Blink)**: Um fluxograma simples mostrando a sequência de chamadas de função (`_start` -> `enable_gpio_clock` -> `setup_gpio` -> `loop` -> `turn_on_led` -> `delay` -> `turn_off_led` -> `delay` -> `loop`).
*   **Diagrama de Registradores GPIO**: Uma representação visual dos bits dentro dos registradores `MODER`, `BSRR` e outros, mostrando como cada bit afeta o comportamento do pino PC13.

Esses diagramas podem ser criados usando ferramentas como draw.io, Inkscape, ou mesmo software de design vetorial, e então exportados como SVG para inclusão em documentos web ou Markdown.

## Conclusão

Programar em Assembly para microcontroladores como o STM32F411 Black Pill é uma habilidade valiosa que oferece controle sem precedentes sobre o hardware e uma compreensão profunda de como os sistemas embarcados funcionam. Embora a curva de aprendizado seja íngreme, os benefícios em termos de otimização, depuração e conhecimento arquitetônico são imensos. Este projeto de exemplo serve como um ponto de partida para sua jornada no mundo da programação de baixo nível para ARM Cortex-M.

Esperamos que este guia detalhado o ajude a explorar e dominar a arte da programação em Assembly para sistemas embarcados. Boa sorte e bons códigos!

---

**Autor:** Manus AI
**Data:** Setembro de 2025


