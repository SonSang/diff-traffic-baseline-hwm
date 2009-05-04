import mpmath
import pylab

class fresnel_sin(object):
    def __init__(self, n):
        self.n = n
        self.denoms = []
        for i in xrange(0, self.n):
            self.denoms.append(mpmath.mpi(1.0/((4*i+3)*mpmath.factorial(2*i+1.0))))
        print self.denoms
    def val(self, x):
        res = 0.0
        sign = 1.0
        for i in xrange(0, self.n):
            res += sign*pow(x, 4*i+3)*self.denoms[i]
        sign *= -1.0
        return res

def fact(x):
    return (1 if x==0 else x * fact(x-1))

if __name__ == '__main__':
    fs = fresnel_sin(20)
    top = 1.0
    size = 400
    x = [0]*size
    y = [0]*size
    for i in xrange(0, size):
        x[i] = top*i/(size-1)
        y[i] = fs.val(x[i])
    pylab.clf()
    pylab.plot(x,y)
    pylab.xlabel('x')
    pylab.ylabel('y')
    pylab.title("Fresnel")
    pylab.show()

