/** @file  sac_compression.c
 *  @brief SPARK Audio Core ADPCM compression / decompression processing stage.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <string.h>
#include "sac_compression.h"

/* CONSTANTS ******************************************************************/
#define EXTEND_MASK_POS_20BITS 0x000FFFFF
#define EXTEND_MASK_NEG_20BITS 0xFFF00000
#define EXTEND_MASK_POS_24BITS 0x00FFFFFF
#define EXTEND_MASK_NEG_24BITS 0xFF000000
#define EXTEND_MASK_POS_32BITS 0xFFFFFFFF
#define EXTEND_MASK_NEG_32BITS 0x00000000

/* MACROS *********************************************************************/
#define NB_SAMPLE_TO_BYTE(nb_sample)        (2 * (nb_sample))
#define BYTE_TO_NB_SAMPLE(nb_byte)          ((nb_byte) / 2)
#define NB_SAMPLE_TO_BYTE_32BITS(nb_sample) (4 * (nb_sample))
#define BYTE_TO_NB_SAMPLE_32BITS(nb_byte)   ((nb_byte) / 4)

/* PRIVATE FUNCTION PROTOTYPES *************************************************/
static void extend_msb_to_32bits(void *instance, uint32_t *value);
static uint16_t pack_stereo(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out);
static uint16_t unpack_stereo(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out);
static uint16_t pack_mono(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out);
static uint16_t unpack_mono(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out);

/* PUBLIC FUNCTIONS ***********************************************************/
void sac_compression_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err)
{
    (void)pipeline;
    (void)mem_pool;

    sac_compression_instance_t *compress_inst = instance;

    *err = SAC_ERR_NONE;

    if (compress_inst == NULL) {
        *err = SAC_ERR_NULL_PTR;
        return;
    }

    if ((compress_inst->bit_depth != SAC_16BITS) &&
        (compress_inst->bit_depth != SAC_20BITS) &&
        (compress_inst->bit_depth != SAC_24BITS) &&
        (compress_inst->bit_depth != SAC_32BITS)) {
        *err = SAC_ERR_BIT_DEPTH;
        return;
    }

    adpcm_init_state(&(compress_inst->_internal.adpcm_left_state));
    adpcm_init_state(&(compress_inst->_internal.adpcm_right_state));
    compress_inst->_internal.bit_shift_16bits = compress_inst->bit_depth - SAC_16BITS;

    switch (compress_inst->compression_mode) {
    case SAC_COMPRESSION_PACK_STEREO:
        if (compress_inst->bit_depth == SAC_16BITS) {
            compress_inst->_internal.discard_size =  NB_SAMPLE_TO_BYTE(2);
        } else {
            compress_inst->_internal.discard_size =  NB_SAMPLE_TO_BYTE_32BITS(2);
        }
        break;
    case SAC_COMPRESSION_PACK_MONO:
        if (compress_inst->bit_depth == SAC_16BITS) {
            compress_inst->_internal.discard_size =  NB_SAMPLE_TO_BYTE(1);
        } else {
            compress_inst->_internal.discard_size =  NB_SAMPLE_TO_BYTE_32BITS(1);
        }
        break;
    case SAC_COMPRESSION_UNPACK_STEREO:
    case SAC_COMPRESSION_UNPACK_MONO:
        compress_inst->_internal.msb_position = compress_inst->bit_depth - 1;
        if (compress_inst->bit_depth == SAC_20BITS) {
            compress_inst->_internal.positive_mask = EXTEND_MASK_POS_20BITS;
            compress_inst->_internal.negative_mask = EXTEND_MASK_NEG_20BITS;
        } else if (compress_inst->bit_depth == SAC_24BITS) {
            compress_inst->_internal.positive_mask = EXTEND_MASK_POS_24BITS;
            compress_inst->_internal.negative_mask = EXTEND_MASK_NEG_24BITS;
        } else if (compress_inst->bit_depth == SAC_32BITS) {
            compress_inst->_internal.positive_mask = EXTEND_MASK_POS_32BITS;
            compress_inst->_internal.negative_mask = EXTEND_MASK_NEG_32BITS;
        }
        break;
    default:
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }
}

uint32_t sac_compression_ctrl(void *instance, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)cmd;
    (void)arg;

    *err = SAC_ERR_NONE;

    return 0;
}

uint16_t sac_compression_process(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in, uint16_t size,
                                 uint8_t *data_out, sac_error_t *err)
{
    (void)pipeline;
    (void)header;

    sac_compression_instance_t *compress_inst = instance;
    uint16_t output_size = 0;

    *err = SAC_ERR_NONE;

    switch (compress_inst->compression_mode) {
    case SAC_COMPRESSION_PACK_STEREO:
        output_size = pack_stereo(instance, data_in, size, data_out);
        break;
    case SAC_COMPRESSION_UNPACK_STEREO:
        output_size = unpack_stereo(instance, data_in, size, data_out);
        break;
    case SAC_COMPRESSION_PACK_MONO:
        output_size = pack_mono(instance, data_in, size, data_out);
        break;
    case SAC_COMPRESSION_UNPACK_MONO:
        output_size = unpack_mono(instance, data_in, size, data_out);
        break;
    }
    return output_size;
}

uint16_t sac_compression_process_discard(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in, uint16_t size,
                                         uint8_t *data_out, sac_error_t *err)
{
    (void)pipeline;
    (void)header;
    (void)data_out;

    uint8_t discard_size;
    uint8_t discard_buffer[10];

    sac_compression_instance_t *compress_inst = instance;

    *err = SAC_ERR_NONE;

    switch (compress_inst->compression_mode) {
    case SAC_COMPRESSION_PACK_STEREO:
        /* Only need the last stereo sample to be added to the history. */
        discard_size = compress_inst->_internal.discard_size;
        data_in += (size - discard_size);
        pack_stereo(instance, data_in, discard_size, discard_buffer);
        break;
    case SAC_COMPRESSION_PACK_MONO:
        /* Only need the last sample to be added to the history. */
        discard_size = compress_inst->_internal.discard_size;
        data_in += (size - discard_size);
        pack_mono(instance, data_in, discard_size, discard_buffer);
        break;
    case SAC_COMPRESSION_UNPACK_STEREO:
    case SAC_COMPRESSION_UNPACK_MONO:
        break;
    }
    return 0;
}

/* PRIVATE FUNCTIONS ***********************************************************/
/** @brief Pack stereo uncompressed stream to stereo compressed stream.
 *
 *  @param[in]  instance        Compression instance.
 *  @param[in]  buffer_in       Array of the uncompressed stereo data.
 *  @param[in]  buffer_in_size  Size in byte of the input array.
 *  @param[out] buffer_out      Array where the compressed stereo stream is written to.
 *  @return written size, in byte, to the output buffer.
 */
static uint16_t pack_stereo(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out)
{
    int16_t *input_buffer_16bits = (int16_t *)buffer_in;
    int32_t *input_buffer_32bits = (int32_t *)buffer_in;
    uint16_t pcm_sample_count;
    uint8_t left_code;
    uint8_t right_code;
    sac_compression_instance_t *compress_inst = instance;

    if (compress_inst->bit_depth == SAC_16BITS) {
        input_buffer_16bits = (int16_t *)buffer_in;
        pcm_sample_count = BYTE_TO_NB_SAMPLE(buffer_in_size);
    } else {
        input_buffer_32bits = (int32_t *)buffer_in;
        pcm_sample_count = BYTE_TO_NB_SAMPLE_32BITS(buffer_in_size);
    }

    /* Set left ADPCM encoder status. */
    memcpy(buffer_out, &(compress_inst->_internal.adpcm_left_state), sizeof(adpcm_state_t));

    /* Set right ADPCM encoder status. */
    memcpy(&buffer_out[sizeof(adpcm_state_t)], &(compress_inst->_internal.adpcm_right_state), sizeof(adpcm_state_t));

    buffer_out += sizeof(sac_compression_adpcm_stereo_header_t);

    /*
     * Since two samples get compressed into a single byte,
     * the loop will work with two samples at the same time (left and right samples).
     */
    if (compress_inst->bit_depth == SAC_16BITS) {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            left_code  = adpcm_encode(*input_buffer_16bits++, &(compress_inst->_internal.adpcm_left_state));
            right_code = adpcm_encode(*input_buffer_16bits++, &(compress_inst->_internal.adpcm_right_state));
            /* Concatenate two ADPCM samples code per byte in the output buffer (4-bit MSB, 4-bit LSB). */
            *buffer_out++ = (left_code & 0x0F) | ((right_code << 4) & 0xF0);
        }
    } else {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            left_code  = adpcm_encode((int16_t)((*input_buffer_32bits++ >> compress_inst->_internal.bit_shift_16bits) & 0xFFFF),
                                      &(compress_inst->_internal.adpcm_left_state));
            right_code = adpcm_encode((int16_t)((*input_buffer_32bits++ >> compress_inst->_internal.bit_shift_16bits) & 0xFFFF),
                                      &(compress_inst->_internal.adpcm_right_state));
            /* Concatenate two ADPCM samples code per byte in the output buffer (4-bit MSB, 4-bit LSB). */
            *buffer_out++ = (left_code & 0x0F) | ((right_code << 4) & 0xF0);
        }
    }

    return (pcm_sample_count / 2) + sizeof(sac_compression_adpcm_stereo_header_t);
}

/** @brief Unpack stereo compressed stream to stereo uncompressed stream.
 *
 *  @param[in]  instance        Compression instance.
 *  @param[in]  buffer_in       Array of the input stereo compressed data.
 *  @param[in]  buffer_in_size  Size in byte of the input array.
 *  @param[out] buffer_out      Array where the uncompressed stereo stream is written to.
 *  @return written size, in byte, to the output buffer.
 */
static uint16_t unpack_stereo(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out)
{
    int32_t *output_buffer_32bits = (int32_t *)buffer_out;
    int16_t *output_buffer_16bits = (int16_t *)buffer_out;
    uint16_t pcm_sample_count;
    sac_compression_instance_t *compress_inst = instance;

    if (compress_inst->bit_depth == SAC_16BITS) {
        output_buffer_16bits = (int16_t *)buffer_out;
    } else {
        output_buffer_32bits = (int32_t *)buffer_out;
    }

    /* Get left ADPCM status. */
    memcpy(&(compress_inst->_internal.adpcm_left_state), buffer_in, sizeof(adpcm_state_t));

    /* Get right ADPCM encoder status. */
    memcpy(&(compress_inst->_internal.adpcm_right_state), &buffer_in[sizeof(adpcm_state_t)], sizeof(adpcm_state_t));

    buffer_in += sizeof(sac_compression_adpcm_stereo_header_t);
    pcm_sample_count = (buffer_in_size - sizeof(sac_compression_adpcm_stereo_header_t)) * 2;

    /*
     * Since two samples are compressed into a single byte, the loop will work with two compressed
     * samples at the same time (left and right samples).
     */
    if (compress_inst->bit_depth == SAC_16BITS) {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            *output_buffer_16bits++ = adpcm_decode(*buffer_in & 0x0F, &(compress_inst->_internal.adpcm_left_state));
            *output_buffer_16bits++ = adpcm_decode((*buffer_in++ >> 4) & 0x0F, &(compress_inst->_internal.adpcm_right_state));
        }
        return NB_SAMPLE_TO_BYTE(pcm_sample_count);
    } else {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            *output_buffer_32bits = (adpcm_decode(*buffer_in & 0x0F, &(compress_inst->_internal.adpcm_left_state)) <<
                                     compress_inst->_internal.bit_shift_16bits);
            extend_msb_to_32bits(instance, (uint32_t *)output_buffer_32bits);
            output_buffer_32bits++;
            *output_buffer_32bits = (adpcm_decode((*buffer_in++ >> 4) & 0x0F, &(compress_inst->_internal.adpcm_right_state)) <<
                                     compress_inst->_internal.bit_shift_16bits);
            extend_msb_to_32bits(instance, (uint32_t *)output_buffer_32bits);
            output_buffer_32bits++;
        }
        return NB_SAMPLE_TO_BYTE_32BITS(pcm_sample_count);
    }
}

/** @brief Pack mono uncompressed stream to mono compressed stream.
 *
 *  @param[in]  instance        Compression instance.
 *  @param[in]  buffer_in       Array of the uncompressed mono data.
 *  @param[in]  buffer_in_size  Size in byte of the input array.
 *  @param[out] buffer_out      Array where the compressed stereo stream is written to.
 *  @return written size, in byte, to the output buffer.
 */
static uint16_t pack_mono(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out)
{
    int16_t *input_buffer_16bits = (int16_t *)buffer_in;
    int32_t *input_buffer_32bits = (int32_t *)buffer_in;
    uint16_t pcm_sample_count;
    sac_compression_instance_t *compress_inst = instance;

    if (compress_inst->bit_depth == SAC_16BITS) {
        pcm_sample_count = BYTE_TO_NB_SAMPLE(buffer_in_size);
    } else {
        pcm_sample_count = BYTE_TO_NB_SAMPLE_32BITS(buffer_in_size);
    }

    /* Set left ADPCM encoder status. */
    memcpy(buffer_out, &(compress_inst->_internal.adpcm_left_state), sizeof(adpcm_state_t));
    buffer_out += sizeof(adpcm_state_t);

    /* Since two samples get compressed into a single byte, the loop will work with two samples at the same time. */
    if (compress_inst->bit_depth == SAC_16BITS) {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            /* Concatenate two ADPCM samples code per byte in the output buffer (4-bit MSB, 4-bit LSB). */
            *buffer_out = adpcm_encode(*input_buffer_16bits++, &(compress_inst->_internal.adpcm_left_state)) & 0x0F;
            *buffer_out++ |= (adpcm_encode(*input_buffer_16bits++, &(compress_inst->_internal.adpcm_left_state)) << 4) & 0xF0;
        }
        /* Manage odd number of samples */
        if (pcm_sample_count & 0x01) {
            *buffer_out  = adpcm_encode(*input_buffer_16bits, &(compress_inst->_internal.adpcm_left_state)) & 0x0F;
        }
    } else {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            /* Concatenate two ADPCM samples code per byte in the output buffer (4-bit MSB, 4-bit LSB). */
            *buffer_out = adpcm_encode((int16_t)((*input_buffer_32bits++ >> compress_inst->_internal.bit_shift_16bits) & 0xFFFF),
                                       &(compress_inst->_internal.adpcm_left_state));
            *buffer_out++ |= (adpcm_encode((int16_t)((*input_buffer_32bits++ >> compress_inst->_internal.bit_shift_16bits) & 0xFFFF),
                                           &(compress_inst->_internal.adpcm_left_state)) << 4) & 0xF0;
        }
        /* Manage odd number of samples. */
        if (pcm_sample_count & 0x01) {
            *buffer_out = adpcm_encode((int16_t)((*input_buffer_32bits >> compress_inst->_internal.bit_shift_16bits) & 0xFFFF),
                                       &(compress_inst->_internal.adpcm_left_state)) & 0x0F;
        }
    }

    return ((pcm_sample_count / 2) + (pcm_sample_count & 0x01)) * sizeof(uint8_t) + sizeof(state_variable_t);
}

/** @brief Unpack mono compressed stream to mono uncompressed stream.
 *
 *  @param[in]  instance        Compression instance.
 *  @param[in]  buffer_in       Array of the input mono compressed data.
 *  @param[in]  buffer_in_size  Size in byte of the input array.
 *  @param[out] buffer_out      Array where the uncompressed stereo stream is written to.
 *  @return written size, in byte, to the output buffer.
 */
static uint16_t unpack_mono(void *instance, uint8_t *buffer_in, uint16_t buffer_in_size, uint8_t *buffer_out)
{
    int32_t *output_buffer_32bits = (int32_t *)buffer_out;
    int16_t *output_buffer_16bits = (int16_t *)buffer_out;
    uint16_t pcm_sample_count;
    sac_compression_instance_t *compress_inst = instance;

    /* Get left ADPCM status. */
    memcpy(&(compress_inst->_internal.adpcm_left_state), buffer_in, sizeof(adpcm_state_t));
    buffer_in += sizeof(adpcm_state_t);

    /* Get number of mono samples. */
    pcm_sample_count = (buffer_in_size - sizeof(adpcm_state_t)) * 2;

    /*
     * Since two samples are compressed into a single byte, the loop will work with two compressed
     * samples at the same time.
     */
    if (compress_inst->bit_depth == SAC_16BITS) {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            *output_buffer_16bits++ = adpcm_decode((*buffer_in & 0x0F), &(compress_inst->_internal.adpcm_left_state));
            *output_buffer_16bits++ = adpcm_decode((*buffer_in++ >> 4) & 0x0F, &(compress_inst->_internal.adpcm_left_state));
        }
        return NB_SAMPLE_TO_BYTE(pcm_sample_count);
    } else {
        for (uint8_t i = 0; i < pcm_sample_count / 2; i++) {
            *output_buffer_32bits = (adpcm_decode((*buffer_in & 0x0F), &(compress_inst->_internal.adpcm_left_state)) <<
                                     compress_inst->_internal.bit_shift_16bits);
            extend_msb_to_32bits(instance, (uint32_t *)output_buffer_32bits);
            output_buffer_32bits++;
            *output_buffer_32bits = (adpcm_decode((*buffer_in++ >> 4) & 0x0F, &(compress_inst->_internal.adpcm_left_state)) <<
                                     compress_inst->_internal.bit_shift_16bits);
            extend_msb_to_32bits(instance, (uint32_t *)output_buffer_32bits);
            output_buffer_32bits++;
        }
        return NB_SAMPLE_TO_BYTE_32BITS(pcm_sample_count);
    }
}

/** @brief Extend value's sign bit into 32-bit word.
 *
 *  @param[in] value  The input value to be extended.
 */
static void extend_msb_to_32bits(void *instance, uint32_t *value)
{
    sac_compression_instance_t *compress_inst = instance;

    if ((*value) & (1 << compress_inst->_internal.msb_position)) {
        /* Negative value. */
        *value |= compress_inst->_internal.negative_mask;
    } else {
        /* Positive value. */
        *value &= compress_inst->_internal.positive_mask;
    }
}
