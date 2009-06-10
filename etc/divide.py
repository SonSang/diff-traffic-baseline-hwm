import numpy
import itertools

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

def powerset(nums):
    card = len(nums)
    allset = 2L**card - 1
    for i in xrange(2L**(card-1)):
        yield (numpy.take(nums, list(whatset(i))), numpy.take(nums, list(whatset(i ^ allset))))

def partitions(nums, slots):
    if slots == 1:
        yield [nums]
        return
    for thisslot, rest in powerset(nums):
        for rest_part in partitions(rest, slots-1):
            yield [thisslot] + rest_part

def best_partition(nums, slots):
    best = (sum(nums), None)
    for part in partitions(nums, slots):
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

def partitions2(nums, slots):
    for i in combinations([0]*(slots-1) + range(1, 2L**len(nums)), slots):
        if sum(i) == 2L**len(nums)-1:
            yield i

