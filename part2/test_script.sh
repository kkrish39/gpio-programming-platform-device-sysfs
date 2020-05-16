echo 1 > /sys/class/HCSR/HCSR_1/trigger
sleep 2
echo "Trigger Pin Set: "
cat /sys/class/HCSR/HCSR_1/trigger

echo 5 > /sys/class/HCSR/HCSR_1/echo 
sleep 2
echo "Echo Pin Set: "
cat /sys/class/HCSR/HCSR_1/echo

echo 8 > /sys/class/HCSR/HCSR_1/number_samples
sleep 2
echo "Number of samples: "
cat /sys/class/HCSR/HCSR_1/number_samples


echo 70 > /sys/class/HCSR/HCSR_1/sampling_period
sleep 2
echo "Sampling period: "
cat /sys/class/HCSR/HCSR_1/sampling_period

echo "Please wait...Writing 4 more values"
echo 1 > /sys/class/HCSR/HCSR_1/enable
sleep 2

echo 1 > /sys/class/HCSR/HCSR_1/enable
sleep 2

echo 1 > /sys/class/HCSR/HCSR_1/enable
sleep 2

echo 1 > /sys/class/HCSR/HCSR_1/enable
sleep 2

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance

echo "Please wait...Writing 1 more value"
echo 1 > /sys/class/HCSR/HCSR_1/enable
sleep 2

echo "Reading distance"
cat /sys/class/HCSR/HCSR_1/distance