﻿//-------------------------------------------------------------------------------------------------
// File : r3d_image.cpp
// Desc : Image.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <r3d_image.h>


namespace {

//-------------------------------------------------------------------------------------------------
//! @brief      ビットマップの圧縮タイプです.
//-------------------------------------------------------------------------------------------------
enum BMP_COMPRESSION_TYPE
{
    BMP_COMPRESSION_RGB       = 0,      // 無圧縮.
    BMP_COMPRESSION_RLE8      = 1,      // RLE圧縮 8 bits/pixel.
    BMP_COMPRESSION_RLE4      = 2,      // RLE圧縮 4 bits/pixel.
    BMP_COMPRESSION_BITFIELDS = 3,      // ビットフィールド.
};

//-------------------------------------------------------------------------------------------------
//! @brief      ビットマップファイルヘッダーです.
//-------------------------------------------------------------------------------------------------
#pragma pack( push, 1 )
struct BMP_FILE_HEADER
{
    unsigned short      Type;           // ファイルタイプ 'BM'
    unsigned int        Size;           // ファイルサイズ.
    unsigned short      Reserved1;      // 予約領域 (0固定).
    unsigned short      Reserved2;      // 予約領域 (0固定).
    unsigned int        OffBits;        // ファイル先頭から画像データまでのオフセット.
};
#pragma pack( pop )

//-------------------------------------------------------------------------------------------------
//! @brief      ビットマップ情報ヘッダーです.
//-------------------------------------------------------------------------------------------------
#pragma pack( push, 1 )
struct BMP_INFO_HEADER
{
    unsigned int        Size;           // ヘッダサイズ (40固定).
    long                Width;          // 画像の横幅.
    long                Height;         // 画像の縦幅.
    unsigned short      Planes;         // プレーン数 (1固定).
    unsigned short      BitCount;       // 1ピクセルあたりのビット数.
    unsigned int        Compression;    // 圧縮形式.
    unsigned int        SizeImage;      // 画像データ部のサイズ.
    long                XPelsPerMeter;  // 横方向の解像度.
    long                YPelsPerMeter;  // 縦方向の解像度.
    unsigned int        ClrUsed;        // 格納されているパレット数.
    unsigned int        ClrImportant;   // 重要なパレットのインデックス.
};
#pragma pack( pop )

inline double clamp(double x)
{ 
    if (x < 0.0)
        return 0.0;
    if (x > 1.0)
        return 1.0;
    return x;
} 

}


bool save_bmp(const char* filename, int width, int height, const double* pixels)
{
    FILE* pFile;
    auto err = fopen_s(&pFile, filename, "wb");
    if (err != 0)
    { return false; }

    BMP_FILE_HEADER fileHeader;
    BMP_INFO_HEADER infoHeader;

    fileHeader.Type      = 'MB';
    fileHeader.Size      = sizeof(BMP_FILE_HEADER) + sizeof(BMP_INFO_HEADER) + ( width * height * 3 );
    fileHeader.Reserved1 = 0;
    fileHeader.Reserved2 = 0;
    fileHeader.OffBits   = sizeof(BMP_FILE_HEADER) + sizeof(BMP_INFO_HEADER);

    infoHeader.Size          = 40;
    infoHeader.Width         = width;
    infoHeader.Height        = height;
    infoHeader.Planes        = 1;
    infoHeader.BitCount      = 24;
    infoHeader.Compression   = BMP_COMPRESSION_RGB;
    infoHeader.SizeImage     = 0;
    infoHeader.XPelsPerMeter = 0;
    infoHeader.YPelsPerMeter = 0;
    infoHeader.ClrUsed       = 0;
    infoHeader.ClrImportant  = 0;

    fwrite(&fileHeader, sizeof(fileHeader), 1, pFile);
    fwrite(&infoHeader, sizeof(infoHeader), 1, pFile);

    const double inv_gamma = 1.0 / 2.2;

    for (auto i = 0; i < height; ++i)
    {
        for (auto j = 0; j < width; ++j)
        {
            auto idx = (i * width * 3) + (j * 3);

            auto fr = pixels[idx + 0];
            auto fg = pixels[idx + 1];
            auto fb = pixels[idx + 2];

            fr = pow( fr, inv_gamma );
            fg = pow( fg, inv_gamma );
            fb = pow( fb, inv_gamma );

            fr = clamp(fr);
            fg = clamp(fg);
            fb = clamp(fb);

            auto r = static_cast<uint8_t>(fr * 255.0 + 0.5);
            auto g = static_cast<uint8_t>(fg * 255.0 + 0.5);
            auto b = static_cast<uint8_t>(fb * 255.0 + 0.5);

            fwrite(&b, sizeof(b), 1, pFile);
            fwrite(&g, sizeof(g), 1, pFile);
            fwrite(&r, sizeof(r), 1, pFile);
        }
    }

    fclose(pFile);
    return true;
}