#ifndef VISIBILITY_SET_H_
#define VISIBILITY_SET_H_

#include <bitset>
#include <numeric>
#include "global_common.h"


class VisibilitySet
{
#if useBitSet
  using Bitset = std::bitset<MAX_COVERAGE_SIZE>;
#else
  using Bitset = std::array<RealNum, MAX_COVERAGE_SIZE>;

#endif
public:
  VisibilitySet();
  VisibilitySet(const VisibilitySet &other);

  VisibilitySet &operator=(const VisibilitySet &other);
#if useBitSet
  bool operator[](Idx i) const;
#else
  RealNum operator[](Idx i) const;
#endif
  bool operator==(const VisibilitySet &other) const;
  bool operator<(const VisibilitySet &other) const;
  bool operator>(const VisibilitySet &other) const;

  void Clear();
  void SetAll();

  bool At(Idx i) const;
  void Insert(Idx i);
  void Insert(const VisibilitySet &other);
  void Remove(const VisibilitySet &other);
  bool IsExpending(const VisibilitySet &other) const;
  bool Contains(const VisibilitySet &other) const;
  bool IsContainedIn(const VisibilitySet &other) const;
#if useBitSet
  SizeType Size() const;
#else
  RealNum Size() const;
#endif
  const Bitset &bitset() const;

  // private:
  Bitset bitset_;
};

#endif // VISIBILITY_SET_H
