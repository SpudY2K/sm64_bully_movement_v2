# BitFS Bully Movement Brute Forcer.
This program accepts the following options:

<pre>
-f &lt;frames&gt;:                 Maximum frames of movement considered.
                             Must be less than internal maximum of 50.
                             Default: 26

-s &lt;min_speed&gt; &lt;max_speed&gt;:  Inclusive range of speeds to consider.
                             Default: 1.5e+07 1e+09

-d &lt;min_dist&gt; &lt;max_dist&gt;:    Inclusive range of allowed distances from starting position.
                             Routes with distance outside of this range will be discarded.
                             Default: 100 1000

-a &lt;min_idx&gt; &lt;max_idx&gt;:      Inclusive range of angles in arctan table to consider.
                             Out of range indices will be clipped.
                             Default: 0 8191

-p &lt;x&gt; &lt;y&gt; &lt;z&gt;:              Starting position of the bully.
                             Default: -1700 -2950 -350

-q &lt;x&gt; &lt;y&gt; &lt;z&gt;:              Position of the track platform.
                             Note: the platform does not move from this position in this simulation.
                             Default: -1594 -3072 0

-r &lt;x&gt; &lt;y&gt; &lt;z&gt;:              Position of the pyramid platform.
                             Default: -1945 -3225 -715

-n &lt;nx&gt; &lt;ny&gt; &lt;nz&gt;:           Normal of the pyramid platform.
                             Note: the platform does not move from this normal in this simulation.
                             Default: 0 1 0

-t:                          Remove the track platform from the simulation.
                             If provided, values of -q will be ignored.
                             Default: off

-u:                          Remove the pyramid platform from the simulation.
                             If provided, values of -r and -s will be ignored.
                             Default: off

-o:                          Path to the output CSV file of solutions.
                             Default: BullyPositions.txt

-v:                          Verbose mode. Prints all parameters used in brute force.
                             Default: off

-h --help:                   Prints this text.
</pre>



# Building:

Build with cmake using the following commands:
```
cmake .
make
```
