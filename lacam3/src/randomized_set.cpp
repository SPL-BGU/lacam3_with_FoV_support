#include "../include/randomized_set.hpp"

bool RandomizedSet::insert(int x)
{
    if (index.count(x)) {
        return false;
    }
    index[x] = values.size();
    values.push_back(x);
    return true;
}

bool RandomizedSet::remove(int x)
{
    auto it = index.find(x);
    if (it == index.end()) {
        return false;
    }
    size_t i = it->second;
    int last = values.back();
    values[i] = last;
    index[last] = i;
    values.pop_back();
    index.erase(it);
    return true;
}

bool RandomizedSet::getRandom(std::mt19937 &gen, int *random_value)
{
    if (random_value == nullptr) {
        throw std::invalid_argument("random_value cannot be nullptr");
    }
    if (values.empty()) {
        return false;
    }
    std::uniform_int_distribution<size_t> dist(0, values.size() - 1);
    *random_value = values[dist(gen)];
    return true;
}

size_t RandomizedSet::size() const { return values.size(); }

bool RandomizedSet::empty() const { return values.empty(); }
