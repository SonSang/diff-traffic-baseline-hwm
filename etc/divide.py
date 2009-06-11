import numpy
import itertools
import time

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper

def max_diff(nums):
    s = numpy.asarray([sum(x) for x in nums])
    return max(s) - min(s)

def whatset(num):
    bignum = long(num)
    mask = 1L
    count = 0
    while mask <= bignum:
        if mask & bignum:
            yield count
        count += 1
        mask = mask << 1L

def powerset_take(nums):
    card = len(nums)
    allset = 2L**card - 1
    for i in xrange(2L**(card-1)):
        yield (numpy.take(nums, list(whatset(i))), numpy.take(nums, list(whatset(i ^ allset))))

def partitions(nums, slots):
    if slots == 1:
        yield [nums]
        return
    for thisslot, rest in powerset_take(nums):
        for rest_part in partitions(rest, slots-1):
            yield [thisslot] + rest_part

@print_timing
def best_partition(pfunc, nums, slots):
    best = (sum(nums), None)
    for part in pfunc(nums, slots):
        md = max_diff(part)
        if md < best[0]:
            best = (md, part)
    return best

def combinations(iterable, r):
    # combinations('ABCD', 2) --> AB AC AD BC BD CD
    # combinations(range(4), 3) --> 012 013 023 123
    pool = tuple(iterable)
    n = len(pool)
    if r > n:
        return
    indices = range(r)
    yield tuple(pool[i] for i in indices)
    while True:
        for i in reversed(range(r)):
            if indices[i] != i + n - r:
                break
        else:
            return
        indices[i] += 1
        for j in range(i+1, r):
            indices[j] = indices[j-1] + 1
        yield tuple(pool[i] for i in indices)

def powerset_comp(nums):
    card = len(nums)
    if card == 0:
        yield []
        return
    allset = 2L**card-1
    for i in xrange(2L**(card)):
        yield (numpy.take(nums, list(whatset(i))), numpy.take(nums, list(whatset(i ^ allset))))

def partitions2(nums, slots):
    if slots == 1:
        yield [list(nums)]
        return
    if len(nums) < 2:
        yield [list(nums)] + [[]] * (slots-1)
        return
    for subset, comp in powerset_comp(nums[1:]):
        first = [[nums[0]] + list(subset)]
        for subpart in partitions2(comp, slots-1):
            yield first + subpart

if __name__ == '__main__':
    print best_partition(partitions2, range(1,5)*2, 5)
    print best_partition(partitions, range(1,5)*2, 5)

