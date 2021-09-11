#include <iostream>

#include "visibility_set.h"

VisibilitySet::VisibilitySet()
{
    Clear();
}
#if useBitSet
VisibilitySet::VisibilitySet(const VisibilitySet &other)
    : bitset_(other.bitset())
{
}

VisibilitySet &VisibilitySet::operator=(const VisibilitySet &other)
{
    if (this != &other)
    {
        bitset_ = other.bitset();
    }

    return *this;
}

bool VisibilitySet::operator[](Idx i) const
{
    return (bitset_[i] == 1);
}

bool VisibilitySet::operator==(const VisibilitySet &other) const
{
    return (bitset_ == other.bitset());
}

bool VisibilitySet::operator<(const VisibilitySet &other) const
{
    return (this->Size() < other.Size());
}

bool VisibilitySet::operator>(const VisibilitySet &other) const
{
    return (this->Size() > other.Size());
}

void VisibilitySet::Clear()
{
    bitset_.reset();
}

void VisibilitySet::SetAll()
{
    bitset_.set();
}

bool VisibilitySet::At(Idx i) const
{
    return (bitset_[i] == 1);
}

void VisibilitySet::Insert(Idx i)
{
    bitset_.set(i, true);
}

void VisibilitySet::Insert(const VisibilitySet &other)
{
    bitset_ |= other.bitset();
}

void VisibilitySet::Remove(const VisibilitySet &other)
{
    bitset_ &= (~other.bitset());
}

bool VisibilitySet::IsExpending(const VisibilitySet &other) const
{
    return (bitset_ != (bitset_ & other.bitset()));
}

bool VisibilitySet::Contains(const VisibilitySet &other) const
{
    return (other.bitset() == (bitset_ & other.bitset()));
}

bool VisibilitySet::IsContainedIn(const VisibilitySet &other) const
{
    return (bitset_ == (bitset_ & other.bitset()));
}

SizeType VisibilitySet::Size() const
{
    return bitset_.count();
}

const VisibilitySet::Bitset &VisibilitySet::bitset() const
{
    return bitset_;
}
#else
VisibilitySet::VisibilitySet(const VisibilitySet &other)
{
    memcpy(&bitset_, &(other.bitset()), sizeof(bitset_));
    // bitset_ = other.bitset();
}

VisibilitySet &VisibilitySet::operator=(const VisibilitySet &other)
{
    if (this != &other)
    {
        // bitset_ = other.bitset();
        memcpy(&bitset_, &other.bitset(), sizeof(bitset_));
    }

    return *this;
}

RealNum VisibilitySet::operator[](Idx i) const
{
    return bitset_[i];
}

bool VisibilitySet::operator==(const VisibilitySet &other) const
{
    // return (bitset_ == other.bitset());
    auto other_bitset_ = other.bitset();
    bool isEqual = true;
    for (size_t i = 0; i < MAX_COVERAGE_SIZE; i++)
    {
        if (std::abs(bitset_[i] - other_bitset_[i]) > 1e-4)
        {
            isEqual = false;
            break;
        }
    }
    // return isEqual;
    //  auto other_bitset_ = other.bitset();
    // for (size_t i = 0; i < bitset_.size(); i++)
    // {
    //     if (other_bitset_[i] > 0.5 && bitset_[i] < 0.5)
    //     {
    //         return false;
    //     }
    //     if (other_bitset_[i] < 0.5 && bitset_[i] > 0.5)
    //     {
    //         return false;
    //     }

    // }

    // return true;
}

bool VisibilitySet::operator<(const VisibilitySet &other) const
{
    return (this->Size() < other.Size());
}

bool VisibilitySet::operator>(const VisibilitySet &other) const
{
    return (this->Size() > other.Size());
}

void VisibilitySet::Clear()
{
    bitset_.fill(0);
}

void VisibilitySet::SetAll()
{
    bitset_.fill(1);
}

bool VisibilitySet::At(Idx i) const
{
    return bitset_[i];
}

void VisibilitySet::Insert(Idx i)
{
    bitset_[i] = 1;
}

void VisibilitySet::Insert(const VisibilitySet &other)
{
    auto other_bitset_ = other.bitset();
    for (size_t i = 0; i < bitset_.size(); i++)
    {
        // if (other_bitset_[i] > 0.5)
        // {
        //    bitset_[i] = 1;
        // }
        //bitset_[i] |=other_bitset_[i];

        bitset_[i] = std::max(bitset_[i], other_bitset_[i]);
        // bitset_[i] = floor(100*std::max(bitset_[i], other_bitset_[i]))/100.0;
    }
}

void VisibilitySet::Remove(const VisibilitySet &other)
{
    // bitset_ &= (~other.bitset());
    // auto other_bitset_ = other.bitset();
    // for (size_t i = 0; i < bitset_.size(); i++)
    // {
    //     if (other_bitset_[i] < 0.5)
    //     {
    //        bitset_[i] = 0;
    //     }

    // }
}

bool VisibilitySet::IsExpending(const VisibilitySet &other) const
{
    // return (bitset_ != (bitset_ & other.bitset()));
    return !IsContainedIn(other);
}

bool VisibilitySet::Contains(const VisibilitySet &other) const
{
    // return (other.bitset() == (bitset_ & other.bitset()));
    // auto other_bitset_ = other.bitset();

    // bool isContians = true;
    // for (size_t i = 0; i < bitset_.size(); i++)
    // {
    //     if ((bitset_[i] < (other_bitset_[i] - 0.5)) )
    //     {
    //         isContians = false;
    //         break;
    //     }
    // }
    // return isContians;
    // return Size() >=  other.Size();
    auto other_bitset_ = other.bitset();

    bool isContians = true;
    for (size_t i = 0; i < bitset_.size(); i++)
    {
        // if (!(other_bitset_[i] == (bitset_[i] & other_bitset_[i])))
        // {
        //     isContians = false;
        //     break;
        // }

        // if (bitset_[i] < (other_bitset_[i] - 1e-6))
        if (bitset_[i] < (other_bitset_[i] - 1e-3))

        {
            isContians = false;
            break;
        }
    }

    return isContians;
}

bool VisibilitySet::IsContainedIn(const VisibilitySet &other) const
{
    // return (bitset_ == (bitset_ & other.bitset()));
    //   auto other_bitset_ = other.bitset();

    // bool isContians = true;
    // for (size_t i = 0; i < bitset_.size(); i++)
    // {
    //     if ((other_bitset_[i] < (bitset_[i] - 0.5)) )
    //     {
    //         isContians = false;
    //         break;
    //     }
    // }
    // return isContians;

    // auto other_bitset_ = other.bitset();

    // bool isContians = true;
    // for (size_t i = 0; i < bitset_.size(); i++)
    // {
    //     if (bitset_[i] < 0.5 && other_bitset_[i] > 0.5)
    //     {
    //         isContians = false;
    //         break;
    //     }

    // }
    // return isContians;
}

RealNum VisibilitySet::Size() const
{
    // int count = sizeof( bitset_ ) / sizeof( bitset_[0] );
    // RealNum sum = std::accumulate( &bitset_, &bitset_ + count, 0 );
    // return ( RealNum ) sum;
    // auto temp = std::accumulate(std::begin(bitset_), std::end(bitset_), 0);
    // return temp;
    RealNum initial_sum = 0;
    for (size_t i = 0; i < bitset_.size(); i++)
    {
        // if (std::abs(visPOI[i]-1) <eps)
        // if (bitset_[i]>0.5)
        // {

        // }
        // initial_sum +=1;
        initial_sum += bitset_[i];
    }
    // int temp = round(initial_sum*100);

    // // double tempM = temp%100;
    // double temp1 = temp/100.0;

    // return temp1; //(SizeType)(initial_sum+0.1);
    return initial_sum;//
    // return floor(100000 * initial_sum) / 100000.0;
}

const VisibilitySet::Bitset &VisibilitySet::bitset() const
{
    return bitset_;
}

#endif //useBitSet