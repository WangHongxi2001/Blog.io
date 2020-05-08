内容截取自：https://community.sw.siemens.com/s/article/introduction-to-filters-fir-versus-iir

内容截取自：https://community.sw.siemens.com/s/article/introduction-to-filters-fir-versus-iir

内容截取自：https://community.sw.siemens.com/s/article/introduction-to-filters-fir-versus-iir

------

![批注 2020-05-02 122801](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 122801.jpg)![批注 2020-05-02 122907](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 122907.jpg)

![批注 2020-05-02 122949](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 122949.jpg)

![批注 2020-05-02 123111](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 123111.jpg)

![批注 2020-05-02 123153](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 123153.jpg)

![批注 2020-05-02 123241](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 123241.jpg)

The FIR methods use different spectral windows when transforming from the frequency to time domain. Some of the window methods include:

- Chebyshev – Has the lowest amount of ripple in stop band, but widest transition band.
- Hamming – Narrow transition zone, smaller ripple than Hanning. Developed by Richard Hamming, who was a member of the Manhattan Project.
- Kaiser – Developed by James Kaiser at Bell Laboratories, the Kaiser window has small amplitude ripple in stop zone, only the wide transition width Chebyshev has lower amplitude ripple.
- Hanning – Narrowest transition band, but large ripple in stop band.
- Rectangular – Largest amount of ripple/lobes, even affects pass band.

![批注 2020-05-02 123338](http://q9lflo44c.bkt.clouddn.com/img/批注 2020-05-02 123338.jpg)

Attributes of the different IIR filter methods:

- Butterworth – Flat response in both the pass and stop band, but has a wide transition zone. First described by British physicist Stephen Butterworth in 1930.
- Inverse Chebyshev – Flat in the pass band, with a narrower transition width than the Butterworth filter, but has ripple in the stop band. If ripple in the stop band is not a problem, might be preferred for a given application over the Butterworth filter.
- Chebyshev – Can have ripple in pass band, but has steeper rolloff than Inverse Chebyshev.
- Cauer – Narrowest transition zone. Ripple in both stop and pass bands. Sometimes called an Elliptic filter.
- Bessel – Sloping amplitude in both the pass and stop band, with a very wide transition zone. The delay versus frequency in the filter is the flattest in this list. The Bessel filter was named for Freidrich Bessel (1784-1846), a German mathematician.