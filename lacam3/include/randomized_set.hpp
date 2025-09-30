#include <random>
#include <stdexcept>
#include <unordered_map>
#include <vector>

/**
 * @brief A data structure that supports inserting, deleting, and getting a
 * random element in amortized O(1) time.
 *
 */
struct RandomizedSet {
  private:
    std::vector<int> values;
    std::unordered_map<int, size_t> index;

  public:
    /**
     * @brief Inserts a value to the set.
     *
     * @param x The value to insert.
     * @return true if the set did not already contain the specified element.
     * @return false otherwise.
     */
    bool insert(int x);

    /**
     * @brief Removes a value from the set.
     *
     * @param x The value to remove.
     * @return true if the set contained the specified element.
     * @return false otherwise.
     */
    bool remove(int x);

    /**
     * @brief Get the Random object.
     *
     * @param gen A random number generator.
     * @param random_value The random value from the set (output parameter).
     * @return bool true if the set is not empty and a random value was
     * returned.
     * @return false if the set is empty and no value was returned.
     * @throws std::invalid_argument if random_value is nullptr.
     */
    bool getRandom(std::mt19937 &gen, int *random_value);

    /**
     * @brief Gets the size of the set.
     *
     * @return size_t The amount of elements in the set.
     */
    size_t size() const;

    /**
     * @brief Checks if the set is empty.
     *
     * @return true if the set is empty.
     * @return false otherwise.
     */
    bool empty() const;
};
