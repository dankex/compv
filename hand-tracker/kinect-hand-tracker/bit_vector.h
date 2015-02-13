#ifndef BIT_VECTOR_H
#define BIT_VECTOR_H

#include <string.h>
#include <math.h>

/* A simple bit-vector. Everyone's written their own-- here's mine.
   It's endian neutral so life is much easier to deal with on multiple
   platforms.  It probably would be more efficient to do the
   operations on words, but I don't want to get into the endian
   issue. Allows you to set / clear and query a value. All one needs
   for sets.

   Walter Bell (wbell@cs.cornell.edu) 
*/
class BitVector 
{
public:
  /* Create a new bit vector with n indices. All bit values will be
     initialized to clear.
  */
  BitVector(int n)
  {
    pvals = new unsigned char[(int)ceil((float)n/8)];
    nvals = n;
    memset(pvals, 0, (int)ceil((float)n/8));
  }

  virtual ~BitVector()
  {
    delete pvals;
  }

  /* Query the value of a bit. Returns 0 if it's not set, non-zero if
     it's currently set.  
  */
  __inline int bit_value(int idx)
  {
    unsigned char block = pvals[idx / 8];
    return (block & (0x80u >> (idx % 8)));
  }

  /* Set the bit at the index, regardless of it's current value.  
   */
  __inline void bit_set(int idx)
  {
    int blockidx = idx / 8;
    
    pvals[blockidx] = pvals[blockidx] | (0x80u >> (idx % 8));
  }

  /* Clear the bit at the index, regardless of it's current value.
   */
  __inline void bit_clear(int idx)
  {
    int blockidx = idx / 8;
    
    pvals[blockidx] = pvals[blockidx] & (~(0x80u >> (idx % 8)));
  }

  /* Reset the vector to it's initial, cleared state
   */
  void reset()
  {
    memset(pvals, 0, (int)ceil((float)nvals/8));
  }

private:
  unsigned char * pvals;
  int nvals;
};

#endif
