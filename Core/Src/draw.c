#include "draw.h"

// 640x480 프레임 버퍼 (각 바이트당 8픽셀)

// 8x9 폰트 데이터 (기본 ASCII 문자들)
const uint8_t font_data[][9] = {
    // 공백 (ASCII 32)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // ! (ASCII 33)
    {0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x20, 0x00},
    // " (ASCII 34)
    {0x00, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // # (ASCII 35)
    {0x00, 0x50, 0x50, 0xF8, 0x50, 0xF8, 0x50, 0x50, 0x00},
    // $ (ASCII 36)
    {0x00, 0x20, 0x78, 0x80, 0x70, 0x08, 0xF0, 0x20, 0x00},
    // % (ASCII 37)
    {0x00, 0xC0, 0xC8, 0x10, 0x20, 0x40, 0x98, 0x18, 0x00},
    // & (ASCII 38)
    {0x00, 0x40, 0xA0, 0x40, 0xA8, 0x90, 0x68, 0x00, 0x00},
    // ' (ASCII 39)
    {0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // ( (ASCII 40)
    {0x00, 0x10, 0x20, 0x40, 0x40, 0x40, 0x20, 0x10, 0x00},
    // ) (ASCII 41)
    {0x00, 0x40, 0x20, 0x10, 0x10, 0x10, 0x20, 0x40, 0x00},
    // * (ASCII 42)
    {0x00, 0x00, 0x20, 0xA8, 0x70, 0xA8, 0x20, 0x00, 0x00},
    // + (ASCII 43)
    {0x00, 0x00, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x00, 0x00},
    // , (ASCII 44)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x40, 0x00},
    // - (ASCII 45)
    {0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00},
    // . (ASCII 46)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00},
    // / (ASCII 47)
    {0x00, 0x08, 0x10, 0x10, 0x20, 0x40, 0x40, 0x80, 0x00},
    // 0 (ASCII 48)
    {0x00, 0x70, 0x88, 0x98, 0xA8, 0xC8, 0x88, 0x70, 0x00},
    // 1 (ASCII 49)
    {0x00, 0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00},
    // 2 (ASCII 50)
    {0x00, 0x70, 0x88, 0x08, 0x30, 0x40, 0x80, 0xF8, 0x00},
    // 3 (ASCII 51)
    {0x00, 0x70, 0x88, 0x08, 0x30, 0x08, 0x88, 0x70, 0x00},
    // 4 (ASCII 52)
    {0x00, 0x10, 0x30, 0x50, 0x90, 0xF8, 0x10, 0x10, 0x00},
    // 5 (ASCII 53)
    {0x00, 0xF8, 0x80, 0xF0, 0x08, 0x08, 0x88, 0x70, 0x00},
    // 6 (ASCII 54)
    {0x00, 0x30, 0x40, 0x80, 0xF0, 0x88, 0x88, 0x70, 0x00},
    // 7 (ASCII 55)
    {0x00, 0xF8, 0x08, 0x10, 0x20, 0x40, 0x40, 0x40, 0x00},
    // 8 (ASCII 56)
    {0x00, 0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x70, 0x00},
    // 9 (ASCII 57)
    {0x00, 0x70, 0x88, 0x88, 0x78, 0x08, 0x10, 0x60, 0x00},
    // : (ASCII 58)
    {0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00},
    // ; (ASCII 59)
    {0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x20, 0x40, 0x00},
    // < (ASCII 60)
    {0x00, 0x10, 0x20, 0x40, 0x80, 0x40, 0x20, 0x10, 0x00},
    // = (ASCII 61)
    {0x00, 0x00, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0x00, 0x00},
    // > (ASCII 62)
    {0x00, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40, 0x00},
    // ? (ASCII 63)
    {0x00, 0x70, 0x88, 0x08, 0x30, 0x20, 0x00, 0x20, 0x00},
    // @ (ASCII 64)
    {0x00, 0x70, 0x88, 0xB8, 0xA8, 0xB8, 0x80, 0x70, 0x00},
    // A (ASCII 65)
    {0x00, 0x70, 0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x00},
    // B (ASCII 66)
    {0x00, 0xF0, 0x88, 0x88, 0xF0, 0x88, 0x88, 0xF0, 0x00},
    // C (ASCII 67)
    {0x00, 0x70, 0x88, 0x80, 0x80, 0x80, 0x88, 0x70, 0x00},
    // D (ASCII 68)
    {0x00, 0xF0, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF0, 0x00},
    // E (ASCII 69)
    {0x00, 0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0xF8, 0x00},
    // F (ASCII 70)
    {0x00, 0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0x00},
    // G (ASCII 71)
    {0x00, 0x70, 0x88, 0x80, 0xB8, 0x88, 0x88, 0x70, 0x00},
    // H (ASCII 72)
    {0x00, 0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88, 0x00},
    // I (ASCII 73)
    {0x00, 0x70, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00},
    // J (ASCII 74)
    {0x00, 0x38, 0x10, 0x10, 0x10, 0x90, 0x90, 0x60, 0x00},
    // K (ASCII 75)
    {0x00, 0x88, 0x90, 0xA0, 0xC0, 0xA0, 0x90, 0x88, 0x00},
    // L (ASCII 76)
    {0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xF8, 0x00},
    // M (ASCII 77)
    {0x00, 0x88, 0xD8, 0xA8, 0x88, 0x88, 0x88, 0x88, 0x00},
    // N (ASCII 78)
    {0x00, 0x88, 0xC8, 0xA8, 0x98, 0x88, 0x88, 0x88, 0x00},
    // O (ASCII 79)
    {0x00, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00},
    // P (ASCII 80)
    {0x00, 0xF0, 0x88, 0x88, 0xF0, 0x80, 0x80, 0x80, 0x00},
    // Q (ASCII 81)
    {0x00, 0x70, 0x88, 0x88, 0x88, 0xA8, 0x90, 0x68, 0x00},
    // R (ASCII 82)
    {0x00, 0xF0, 0x88, 0x88, 0xF0, 0xA0, 0x90, 0x88, 0x00},
    // S (ASCII 83)
    {0x00, 0x70, 0x88, 0x80, 0x70, 0x08, 0x88, 0x70, 0x00},
    // T (ASCII 84)
    {0x00, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00},
    // U (ASCII 85)
    {0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00},
    // V (ASCII 86)
    {0x00, 0x88, 0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x00},
    // W (ASCII 87)
    {0x00, 0x88, 0x88, 0x88, 0x88, 0xA8, 0xD8, 0x88, 0x00},
    // X (ASCII 88)
    {0x00, 0x88, 0x50, 0x20, 0x20, 0x50, 0x88, 0x88, 0x00},
    // Y (ASCII 89)
    {0x00, 0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20, 0x00},
    // Z (ASCII 90)
    {0x00, 0xF8, 0x08, 0x10, 0x20, 0x40, 0x80, 0xF8, 0x00}
};

// 폰트 정보
#define FONT_WIDTH 8
#define FONT_HEIGHT 9
#define FONT_START_CHAR 32  // 공백부터 시작
#define FONT_END_CHAR 90    // Z까지

// 호환성을 위한 font_A (기존 코드가 깨지지 않도록)
const uint8_t font_A[] = {
    0x00, 0x70, 0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x00
};

/**
 * 비트맵 문자를 지정된 크기로 스케일링하여 화면에 그리기
 *
 * @param font_data: 원본 폰트 데이터 (8픽셀 폭)
 * @param font_height: 원본 폰트 높이
 * @param x: 화면상 X 좌표 (0~639)
 * @param y: 화면상 Y 좌표 (0~479)
 * @param scale_x: X축 스케일 배수
 * @param scale_y: Y축 스케일 배수
 */
void draw_scaled_char(const uint8_t* font_data, uint8_t font_height,
                     uint16_t x, uint16_t y, uint8_t scale_x, uint8_t scale_y) {

    // 경계 검사
    if (x >= 640 || y >= 480) return;

    uint16_t scaled_width = 8 * scale_x;
    uint16_t scaled_height = font_height * scale_y;

    // 화면을 벗어나는 경우 클리핑
    if (x + scaled_width > 640) scaled_width = 640 - x;
    if (y + scaled_height > 480) scaled_height = 480 - y;

    // 각 원본 픽셀을 스케일링하여 그리기
    for (uint8_t row = 0; row < font_height && (y + row * scale_y) < 480; row++) {
        uint8_t font_byte = font_data[row];

        // Y축 스케일링
        for (uint8_t sy = 0; sy < scale_y && (y + row * scale_y + sy) < 480; sy++) {
            uint16_t target_y = y + row * scale_y + sy;

            // 8개 픽셀을 각각 처리
            for (uint8_t bit = 0; bit < 8; bit++) {
                if (font_byte & (0x80 >> bit)) { // 해당 비트가 1이면
                    uint16_t pixel_x = x + bit * scale_x;

                    // X축 스케일링
                    for (uint8_t sx = 0; sx < scale_x && (pixel_x + sx) < 640; sx++) {
                        uint16_t target_x = pixel_x + sx;
                        uint8_t byte_idx = target_x / 8;
                        uint8_t bit_idx = 7 - (target_x % 8);

                        fb[target_y][byte_idx] |= (1 << bit_idx);
                    }
                }
            }
        }
    }
}

/**
 * 최적화된 버전 - 정수 스케일링만 지원 (더 빠름)
 */
void draw_scaled_char_fast(const uint8_t* font_data, uint8_t font_height,
                          uint16_t x, uint16_t y, uint8_t scale) {

    if (x >= 640 || y >= 480 || scale == 0) return;

    // 사전 계산
    uint16_t scaled_width = 8 * scale;
    uint16_t scaled_height = font_height * scale;

    if (x + scaled_width > 640 || y + scaled_height > 480) {
        // 경계를 벗어나면 일반 함수 호출
        draw_scaled_char(font_data, font_height, x, y, scale, scale);
        return;
    }

    // 경계 체크가 필요없는 최적화된 루프
    for (uint8_t row = 0; row < font_height; row++) {
        uint8_t font_byte = font_data[row];

        if (font_byte == 0) {
            // 빈 행은 건너뛰기
            continue;
        }

        for (uint8_t sy = 0; sy < scale; sy++) {
            uint16_t target_y = y + row * scale + sy;

            for (uint8_t bit = 0; bit < 8; bit++) {
                if (font_byte & (0x80 >> bit)) {
                    uint16_t pixel_x = x + bit * scale;

                    // 스케일된 픽셀 블록 그리기
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        uint16_t target_x = pixel_x + sx;
                        uint8_t byte_idx = target_x >> 3; // /8과 동일하지만 더 빠름
                        uint8_t bit_idx = 7 - (target_x & 7); // %8과 동일하지만 더 빠름

                        fb[target_y][byte_idx] |= (1 << bit_idx);
                    }
                }
            }
        }
    }
}

/**
 * 메모리 효율적인 버전 - 큰 스케일링에 적합
 */
void draw_scaled_char_memory_efficient(const uint8_t* font_data, uint8_t font_height,
                                      uint16_t x, uint16_t y, uint8_t scale) {

    if (x >= 640 || y >= 480 || scale == 0) return;

    // 행별로 처리하여 캐시 효율성 향상
    for (uint8_t row = 0; row < font_height; row++) {
        uint8_t font_byte = font_data[row];

        if (font_byte == 0) continue; // 빈 행 건너뛰기

        uint16_t base_y = y + row * scale;
        if (base_y >= 480) break;

        // 현재 행의 스케일된 버전을 임시 버퍼에 생성
        uint8_t temp_row[80] = {0}; // 최대 640픽셀 = 80바이트

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (font_byte & (0x80 >> bit)) {
                uint16_t pixel_x = x + bit * scale;

                for (uint8_t sx = 0; sx < scale && (pixel_x + sx) < 640; sx++) {
                    uint16_t target_x = pixel_x + sx;
                    uint8_t byte_idx = target_x >> 3;
                    uint8_t bit_idx = 7 - (target_x & 7);

                    temp_row[byte_idx] |= (1 << bit_idx);
                }
            }
        }

        // 생성된 행을 Y축으로 복사
        for (uint8_t sy = 0; sy < scale && (base_y + sy) < 480; sy++) {
            for (uint8_t i = 0; i < 80; i++) {
                fb[base_y + sy][i] |= temp_row[i];
            }
        }
    }
}

/**
 * 문자의 폰트 데이터를 가져오는 함수
 * @param ch: ASCII 문자
 * @return: 폰트 데이터 포인터 (없으면 NULL)
 */
const uint8_t* get_char_font(char ch) {
    if (ch >= FONT_START_CHAR && ch <= FONT_END_CHAR) {
        return font_data[ch - FONT_START_CHAR];
    }
    return NULL; // 지원하지 않는 문자
}

/**
 * 단일 문자를 화면에 그리기
 * @param ch: 그릴 문자
 * @param x: X 좌표
 * @param y: Y 좌표
 * @param scale: 스케일 배수
 * @return: 그려진 문자의 폭 (다음 문자 위치 계산용)
 */
uint16_t draw_char(char ch, uint16_t x, uint16_t y, uint8_t scale) {
    const uint8_t* char_font = get_char_font(ch);
    if (char_font == NULL) {
        // 지원하지 않는 문자는 공백으로 처리
        return FONT_WIDTH * scale;
    }

    draw_scaled_char_fast(char_font, FONT_HEIGHT, x, y, scale);
    return FONT_WIDTH * scale;
}

/**
 * 문자열을 화면에 그리기 (기본)
 * @param str: 출력할 문자열
 * @param x: 시작 X 좌표
 * @param y: 시작 Y 좌표
 * @param scale: 스케일 배수
 * @return: 그려진 문자열의 총 폭
 */
uint16_t draw_string(const char* str, uint16_t x, uint16_t y, uint8_t scale) {
    if (str == NULL || scale == 0) return 0;

    uint16_t current_x = x;
    uint16_t char_spacing = 1 * scale; // 문자 간 간격

    while (*str) {
        if (current_x >= 640) break; // 화면을 벗어나면 중단

        uint16_t char_width = draw_char(*str, current_x, y, scale);
        current_x += char_width + char_spacing;
        str++;
    }

    return current_x - x - char_spacing; // 마지막 간격 제거
}

/**
 * 문자열을 화면에 그리기 (고급 - 줄바꿈 지원)
 * @param str: 출력할 문자열
 * @param x: 시작 X 좌표
 * @param y: 시작 Y 좌표
 * @param scale: 스케일 배수
 * @param max_width: 최대 폭 (0이면 화면 끝까지)
 * @param line_spacing: 줄 간격 (픽셀 단위, 0이면 자동)
 * @return: 그려진 총 높이
 */
uint16_t draw_string_wrapped(const char* str, uint16_t x, uint16_t y, uint8_t scale,
                            uint16_t max_width, uint8_t line_spacing) {
    if (str == NULL || scale == 0) return 0;

    uint16_t start_x = x;
    uint16_t current_x = x;
    uint16_t current_y = y;
    uint16_t char_width = FONT_WIDTH * scale;
    uint16_t char_height = FONT_HEIGHT * scale;
    uint16_t char_spacing = 1 * scale;

    if (max_width == 0) max_width = 640 - x;
    if (line_spacing == 0) line_spacing = 2 * scale;

    while (*str) {
        // 줄바꿈 문자 처리
        if (*str == '\n') {
            current_x = start_x;
            current_y += char_height + line_spacing;
            if (current_y >= 480) break;
            str++;
            continue;
        }

        // 현재 줄에서 문자가 들어갈 공간이 있는지 확인
        if (current_x + char_width > start_x + max_width) {
            // 줄바꿈
            current_x = start_x;
            current_y += char_height + line_spacing;
            if (current_y >= 480) break;
        }

        draw_char(*str, current_x, current_y, scale);
        current_x += char_width + char_spacing;
        str++;
    }

    return (current_y + char_height) - y;
}

/**
 * 중앙 정렬로 문자열 그리기
 * @param str: 출력할 문자열
 * @param center_x: 중앙 X 좌표
 * @param y: Y 좌표
 * @param scale: 스케일 배수
 * @return: 그려진 문자열의 폭
 */
uint16_t draw_string_centered(const char* str, uint16_t center_x, uint16_t y, uint8_t scale) {
    if (str == NULL) return 0;

    // 문자열 길이 계산
    uint16_t len = 0;
    const char* temp = str;
    while (*temp++) len++;

    uint16_t char_width = FONT_WIDTH * scale;
    uint16_t char_spacing = 1 * scale;
    uint16_t total_width = len * char_width + (len - 1) * char_spacing;

    uint16_t start_x = center_x - total_width / 2;
    return draw_string(str, start_x, y, scale);
}

/**
 * 우측 정렬로 문자열 그리기
 * @param str: 출력할 문자열
 * @param right_x: 우측 끝 X 좌표
 * @param y: Y 좌표
 * @param scale: 스케일 배수
 * @return: 그려진 문자열의 폭
 */
uint16_t draw_string_right_aligned(const char* str, uint16_t right_x, uint16_t y, uint8_t scale) {
    if (str == NULL) return 0;

    // 문자열 길이 계산
    uint16_t len = 0;
    const char* temp = str;
    while (*temp++) len++;

    uint16_t char_width = FONT_WIDTH * scale;
    uint16_t char_spacing = 1 * scale;
    uint16_t total_width = len * char_width + (len - 1) * char_spacing;

    uint16_t start_x = right_x - total_width;
    return draw_string(str, start_x, y, scale);
}

/**
 * 프레임 버퍼 영역을 지우기
 * @param x: 시작 X 좌표
 * @param y: 시작 Y 좌표
 * @param width: 폭
 * @param height: 높이
 */
void clear_region(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    for (uint16_t py = y; py < y + height && py < 480; py++) {
        for (uint16_t px = x; px < x + width && px < 640; px++) {
            uint8_t byte_idx = px / 8;
            uint8_t bit_idx = 7 - (px % 8);
            fb[py][byte_idx] &= ~(1 << bit_idx);
        }
    }
}

/**
 * 프레임 버퍼 내용을 콘솔에 출력하는 함수
 * @param start_x: 출력할 시작 X 좌표
 * @param start_y: 출력할 시작 Y 좌표
 * @param width: 출력할 폭 (픽셀 단위)
 * @param height: 출력할 높이 (픽셀 단위)
 */
void print_fb_region(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height) {
    printf("FB region (%d,%d) size %dx%d:\n", start_x, start_y, width, height);
    printf("+");
    for (uint16_t i = 0; i < width; i++) printf("-");
    printf("+\n");

    for (uint16_t y = start_y; y < start_y + height && y < 480; y++) {
        printf("|");
        for (uint16_t x = start_x; x < start_x + width && x < 640; x++) {
            uint8_t byte_idx = x / 8;
            uint8_t bit_idx = 7 - (x % 8);
            uint8_t pixel = (fb[y][byte_idx] >> bit_idx) & 1;
            printf("%c", pixel ? '#' : ' ');
        }
        printf("|\n");
    }

    printf("+");
    for (uint16_t i = 0; i < width; i++) printf("-");
    printf("+\n\n");
}

/**
 * 전체 프레임 버퍼를 축소하여 콘솔에 출력 (큰 화면용)
 * @param scale_down: 축소 비율 (1=원본크기, 2=1/2크기, 4=1/4크기 등)
 */
void print_fb_scaled(uint8_t scale_down) {
    if (scale_down == 0) scale_down = 1;

    uint16_t display_width = 640 / scale_down;
    uint16_t display_height = 480 / scale_down;

    printf("Full framebuffer (1/%d scale):\n", scale_down);
    printf("+");
    for (uint16_t i = 0; i < display_width; i++) printf("-");
    printf("+\n");

    for (uint16_t dy = 0; dy < display_height; dy++) {
        printf("|");
        for (uint16_t dx = 0; dx < display_width; dx++) {
            // scale_down x scale_down 영역에서 하나라도 픽셀이 있으면 출력
            uint8_t has_pixel = 0;
            for (uint8_t sy = 0; sy < scale_down && !has_pixel; sy++) {
                for (uint8_t sx = 0; sx < scale_down && !has_pixel; sx++) {
                    uint16_t x = dx * scale_down + sx;
                    uint16_t y = dy * scale_down + sy;
                    if (x < 640 && y < 480) {
                        uint8_t byte_idx = x / 8;
                        uint8_t bit_idx = 7 - (x % 8);
                        if ((fb[y][byte_idx] >> bit_idx) & 1) {
                            has_pixel = 1;
                        }
                    }
                }
            }
            printf("%c", has_pixel ? '#' : ' ');
        }
        printf("|\n");
    }

    printf("+");
    for (uint16_t i = 0; i < display_width; i++) printf("-");
    printf("+\n\n");
}

/**
 * 프레임 버퍼의 특정 영역만 간단히 출력 (디버깅용)
 */
void print_fb_simple(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height) {
    printf("FB (%d,%d) %dx%d:\n", start_x, start_y, width, height);

    for (uint16_t y = start_y; y < start_y + height && y < 480; y++) {
        for (uint16_t x = start_x; x < start_x + width && x < 640; x++) {
            uint8_t byte_idx = x / 8;
            uint8_t bit_idx = 7 - (x % 8);
            uint8_t pixel = (fb[y][byte_idx] >> bit_idx) & 1;
            printf("%c", pixel ? '#' : '.');
        }
        printf("\n");
    }
    printf("\n");
}

// 사용 예시와 메인 함수
void example_usage(void) {
    // 화면 초기화
    memset(fb, 0, sizeof(fb));

    printf("=== 문자열 출력 테스트 ===\n\n");

    // 1. 기본 문자열 출력
    draw_string("HELLO WORLD!", 10, 10, 3);
    printf("1. Basic string at (10,10) scale 1:\n");
    print_fb_region(5, 5, 100, 20);

    // 2. 큰 크기 문자열
    draw_string("BIG TEXT", 10, 40, 4);
    printf("2. Scaled string at (10,40) scale 2:\n");
    print_fb_region(5, 35, 120, 25);

    // 3. 중앙 정렬
    draw_string_centered("CENTERED", 320, 80, 3);
    printf("3. Centered string at x=320:\n");
    print_fb_region(250, 75, 140, 15);

//    // 4. 우측 정렬
//    draw_string_right_aligned("RIGHT", 630, 100, 3);
//    printf("4. Right aligned string:\n");
//    print_fb_region(580, 95, 60, 15);
//
//    // 5. 줄바꿈이 있는 문자열
//    clear_region(10, 120, 200, 50);
//    draw_string_wrapped("THIS IS A LONG STRING\nWITH LINE BREAKS", 10, 120, 3, 150, 0);
//    printf("5. Multi-line string with wrapping:\n");
//    print_fb_region(5, 115, 160, 40);
//
//    // 6. 숫자 출력 테스트
//    draw_string("NUMBERS: 0123456789", 10, 170, 3);
//    printf("6. Numbers test:\n");
//    print_fb_region(5, 165, 200, 15);
//
//    // 7. 특수 문자 테스트
//    draw_string("SYMBOLS: !@#$%^&*()", 10, 190, 3);
//    printf("7. Symbols test:\n");
//    print_fb_region(5, 185, 200, 15);
//
//    // 8. 전체 화면 미리보기
//    printf("8. Full screen overview (1/4 scale):\n");
//    print_fb_scaled(4);
}

void string_demo(void) {
    printf("\n=== 고급 문자열 데모 ===\n");

    // 화면 초기화
    memset(fb, 0, sizeof(fb));

    // 제목
    draw_string_centered("STM32F411RE", 320, 10, 3);
    draw_string_centered("BITMAP DISPLAY DEMO", 320, 40, 1);

    // 정보 표시
    draw_string("RESOLUTION: 640X480", 10, 70, 1);
    draw_string("FONT SIZE: 8X9 PIXELS", 10, 85, 1);
    draw_string("MEMORY: 38400 BYTES", 10, 100, 1);

    // 스케일 테스트
    draw_string("SCALE 1X", 10, 130, 1);
    draw_string("SCALE 2X", 10, 150, 2);
    draw_string("SCALE 3X", 10, 180, 3);

    // 우측에 정렬 테스트
    draw_string_right_aligned("RIGHT 1", 630, 130, 1);
    draw_string_right_aligned("RIGHT 2", 630, 145, 1);
    draw_string_right_aligned("RIGHT 3", 630, 160, 1);

    // 중앙 정렬 테스트
    draw_string_centered("CENTER 1", 320, 250, 1);
    draw_string_centered("CENTER 2", 320, 265, 2);

    // 줄바꿈 테스트
    draw_string_wrapped("THIS IS A VERY LONG TEXT THAT WILL BE AUTOMATICALLY WRAPPED TO MULTIPLE LINES WHEN IT EXCEEDS THE SPECIFIED WIDTH LIMIT.",
                       10, 300, 1, 300, 2);

    printf("Advanced string demo:\n");
    print_screen_scaled(2);
}
