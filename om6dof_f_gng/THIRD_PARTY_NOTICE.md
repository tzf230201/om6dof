# Third-party notice

F-GNG builds on the DBL-GNG batch learning scheme and its distributed seeding.
Those parts are a C++ adaptation of:

> Chyan Zheng Siow, Azhar Aulia Saputra, Takenori Obo, and Naoyuki Kubota,
> "Distributed Batch Learning of Growing Neural Gas for Quick and Efficient
> Clustering", Mathematics 12(12), 1909, 2024.

Reference implementation: https://github.com/CornerSiow/DBL-GNG

The density target, the merge operator, persistent node identity and the
split/merge hysteresis in `include/fgng_core.hpp` are not part of DBL-GNG.

MIT License

Copyright (c) 2024 Corner Siow

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
