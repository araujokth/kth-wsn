/*
 * Copyright (c) 2008 Johns Hopkins University.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written
 * agreement is hereby granted, provided that the above copyright
 * notice, the (updated) modification history and the author appear in
 * all copies of this source code.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, LOSS OF USE, DATA,
 * OR PROFITS) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * @author Chieh-Jan Mike Liang <cliang4@cs.jhu.edu>
 * @author Kevin Klues <klueska@cs.stanford.edu>
 */

module BitArrayUtilsC {
  provides interface BitArrayUtils;
}

implementation {
  uint16_t getByteIndex(uint8_t bitIndex) {
    return bitIndex / 8;
  }
  uint8_t getMask(uint8_t bitIndex) {
    return 1 << (bitIndex % 8);
  }
  async command void BitArrayUtils.clrArray(uint8_t* array, uint8_t size) {
    memset(array, 0, size);
  }
  async command bool BitArrayUtils.getBit(uint8_t* array, uint8_t bitIndex) {
    return (array[getByteIndex(bitIndex)] & getMask(bitIndex)) ? TRUE : FALSE;
  }
  async command void BitArrayUtils.setBit(uint8_t* array, uint8_t bitIndex) {
    array[getByteIndex(bitIndex)] |= getMask(bitIndex);
  }
  async command void BitArrayUtils.clrBit(uint8_t* array, uint8_t bitIndex) {
    array[getByteIndex(bitIndex)] &= ~getMask(bitIndex);
  }
}
