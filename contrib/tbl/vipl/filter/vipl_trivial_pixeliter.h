// This class provides a trivial "iterator" class for use as the default pixel
// iterator in the new filter paradigm.  Any pixel iterator must provide a
// cast-to integer method (for the appropriate "axis").  The class has
// typedefs, to int, for the 4 "iterators" that we expect to be used.
// It has little functionality, and also serves as an example/test.

#ifndef vipl_trivial_pixeliter_h_
#define vipl_trivial_pixeliter_h_

class vipl_trivial_pixeliter{
    public: typedef int Titerator;
    public: typedef int Xiterator;
    public: typedef int Yiterator;
    public: typedef int Ziterator;
};
#endif
