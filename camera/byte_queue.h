#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <iostream>

#include <condition_variable>
#include <mutex>
constexpr auto DEFAULT_DATA_QUEUE_SIZE = 1024 * 1024 * 16;  

class DataByteQueue {
 private:
  std::mutex mu_;
  std::mutex cond_mu_;
  std::condition_variable_any cond_put_;

 private:
  uint8_t* data_;
  int max_size_;
  int current_pos_;

  uint8_t* data_tmp_;
  int data_tmp_size_;

 public:
  DataByteQueue(int maxsize = DEFAULT_DATA_QUEUE_SIZE) {
    data_ = new uint8_t[maxsize];
    max_size_ = maxsize;
    current_pos_ = 0;
    data_tmp_size_ = maxsize;
    data_tmp_ = new uint8_t[data_tmp_size_];
  }

  virtual ~DataByteQueue() {
    if (data_) {
      delete[] data_;
      data_ = nullptr;
    }
    if (data_tmp_) {
      delete data_tmp_;
      data_tmp_ = nullptr;
    }
  }

  bool addData(const uint8_t* data, int length) {
    std::unique_lock<std::mutex> lock(mu_);
    bool state = addData_(data, length);
    cond_put_.notify_one();
    return state;
  }


  void waitToSize(int size) {
    while (current_pos_ < size) {
      std::unique_lock<std::mutex> cond_lock(cond_mu_);
      cond_put_.wait(cond_lock);
    }
  }

  std::mutex& getMutex() { return mu_; }

  uint8_t* getData() { return data_; }

  inline int getCurrentSize() const { return current_pos_; }

  inline int getMaxSize() const { return max_size_; }

  void clearBuffer() { current_pos_ = 0; }

  void readOver(int size) {
    // memcpy();
    assert(size <= current_pos_);
    int leftSize = current_pos_ - size;

    //std::cout << "leftsize:" << leftSize << std::endl;
    // if(leftSize > 0)
    {
      assert(leftSize <= data_tmp_size_);
      memcpy(data_tmp_, data_ + size, leftSize);
      memcpy(data_, data_tmp_, leftSize);
      current_pos_ = leftSize;
      //std::cout<< "read over : left Size =" << leftSize << "-------"<< std::endl;
    }
  }


 private:
  bool addData_(const uint8_t* data, int length) {
    if ((current_pos_ + length) > max_size_) {
      //超过长度
      int newLength = std::max(max_size_ * 2, current_pos_ + length);
      uint8_t* newData = new uint8_t[newLength];
      memcpy(newData, data_, current_pos_);
      delete[] data_;
      data_ = newData;
      max_size_ = newLength;

      //
      data_tmp_size_ = max_size_;
      delete[] data_tmp_;
      data_tmp_ = new uint8_t[data_tmp_size_];
    }

    memcpy(data_ + current_pos_, data, length);
    current_pos_ += length;

    return true;
  }
};
