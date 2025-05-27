#include <optional>
#include <unordered_map>

template <typename K, typename V>
class MaxKeyMap
{
  private:
    std::unordered_map<K, V> map;
    std::optional<K> maxKey;

    void recomputeMaxKey()
    {
        maxKey.reset();
        for (const auto& pair : map) {
            if (!maxKey || pair.first > *maxKey) {
                maxKey = pair.first;
            }
        }
    }

  public:
    void insert(const K& key, const V& value)
    {
        map[key] = value;
        if (!maxKey || key > *maxKey) {
            maxKey = key;
        }
    }

    void erase(const K& key)
    {
        if (map.erase(key)) {
            if (maxKey && key == *maxKey) {
                recomputeMaxKey();
            }
        }
    }

    std::optional<K> getMaxKey() const { return maxKey; }

    bool contains(const K& key) const { return map.find(key) != map.end(); }

    std::unordered_map<K, V>& getMap() { return map; }
};
