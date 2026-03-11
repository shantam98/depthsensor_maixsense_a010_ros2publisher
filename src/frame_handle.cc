#include <string.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "frame_struct.h"

frame_t *handle_process(std::string s) {
  static std::vector<uint8_t> vecChar;
  static const uint8_t sflag_l = FRAME_BEGIN_FLAG & 0xff;
  static const uint8_t sflag_h = (FRAME_BEGIN_FLAG >> 8) & 0xff;
  static const uint8_t eflag = FRAME_END_FLAG & 0xff;

  uint32_t frame_payload_len = 0;
  frame_t *pf = NULL;
  std::vector<uint8_t>::iterator it;

  vecChar.insert(vecChar.end(), s.cbegin(), s.cend());
  std::cerr << "[FRAME] handle_process called, new input=" << s.size()
            << " bytes, buffer total=" << vecChar.size() << " bytes" << std::endl;

  if (vecChar.size() < 2) {
    std::cerr << "[FRAME] data is not enough!" << std::endl;
    goto __finished;
  }

__find_header:
  it = vecChar.begin();
  do {
    it = find(it + 1, vecChar.end(), sflag_h);
    if (it == vecChar.end()) {
      /* keep last element which may be sflag_l */
      std::vector<uint8_t>(vecChar.end() - 1, vecChar.end()).swap(vecChar);
      std::cerr << "[FRAME] frame head not found! wait more data." << std::endl;
      goto __finished;
    }
  } while (*(it - 1) != sflag_l);
  /* we got *it==sflag_h and *(it-1)==sflag_l */

  /* move frame start (it-1) to front */
  if (it - 1 != vecChar.begin()) {
    std::vector<uint8_t>(it - 1, vecChar.end()).swap(vecChar);
    std::cerr << "[FRAME] frame moved to front (skipped junk bytes)" << std::endl;
    /* after the swap, the header is now at begin(); update 'it' accordingly */
    it = vecChar.begin() + 1; /* it pointed to sflag_h, which is now index 1 */
  }

  if (vecChar.size() < sizeof(frame_t)) {
    std::cerr << "[FRAME] frame head data is not enough now ("
              << vecChar.size() << " < " << sizeof(frame_t) << ")! wait more data." << std::endl;
    goto __finished;
  }

  pf = (frame_t *)&vecChar[0];
  frame_payload_len = pf->frame_head.frame_data_len - FRAME_HEAD_DATA_SIZE;
  std::cerr << "[FRAME] header found: frame_data_len=" << pf->frame_head.frame_data_len
            << " payload_len=" << frame_payload_len
            << " rows=" << (int)pf->frame_head.resolution_rows
            << " cols=" << (int)pf->frame_head.resolution_cols
            << " frame_id=" << (int)pf->frame_head.frame_id << std::endl;

  /* max frame payload size */
  if (frame_payload_len > 100 * 100) {
    std::cerr << "[FRAME] payload_len=" << frame_payload_len << " exceeds max (10000), skipping header" << std::endl;
    /* FIX: skip past this bad header (begin+1) before searching again */
    std::vector<uint8_t>(vecChar.begin() + 2, vecChar.end()).swap(vecChar);
    goto __find_header;
  }

  {
    const size_t full_frame_size = FRAME_HEAD_SIZE + frame_payload_len +
                                   FRAME_CHECKSUM_SIZE + FRAME_END_SIZE;
    if (vecChar.size() < full_frame_size) {
      std::cerr << "[FRAME] expected frame payload length: " << frame_payload_len << std::endl;
      std::cerr << "[FRAME] frame payload data is not enough now ("
                << vecChar.size() << " bytes in buffer, need "
                << full_frame_size << ")! wait more data." << std::endl;
      goto __finished;
    }
  }

  {
    uint8_t check_sum = 0;
    for (uint32_t i = 0; i < FRAME_HEAD_SIZE + frame_payload_len; i++) {
      check_sum += ((uint8_t *)pf)[i];
    }
    uint8_t expected_cs  = ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len];
    uint8_t actual_eflag = ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE];

    if (check_sum != expected_cs || eflag != actual_eflag) {
      std::cerr << "[FRAME] checksum or tail invalid! computed=" << (int)check_sum
                << " expected=" << (int)expected_cs
                << " tail_byte=0x" << std::hex << (int)actual_eflag
                << " expected_tail=0x" << (int)eflag << std::dec
                << ". Retrying header search." << std::endl;
      /* FIX: skip past begin+1 (past sflag_l) so we don't re-match the same header */
      std::vector<uint8_t>(vecChar.begin() + 2, vecChar.end()).swap(vecChar);
      goto __find_header;
    }
  }

  {
    const size_t full_frame_size = FRAME_HEAD_SIZE + frame_payload_len +
                                   FRAME_CHECKSUM_SIZE + FRAME_END_SIZE;

    pf = (frame_t *)malloc(sizeof(frame_t) + frame_payload_len);
    if (!pf) {
      std::cerr << "[FRAME] malloc failed!" << std::endl;
      goto __finished;
    }
    memcpy(pf, &vecChar[0], sizeof(frame_t) + frame_payload_len);

    /* FIX: advance from vecChar.begin() by the full frame size, not from 'it' */
    std::vector<uint8_t>(vecChar.begin() + full_frame_size, vecChar.end())
        .swap(vecChar);
    return pf;
  }

__finished:
  return NULL;
}