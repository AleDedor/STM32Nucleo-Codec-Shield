#ifndef FILTER_H
#define FILTER_H

// even though we just need to filter uint16_t, let's use templates for the defined class
// defined input_t, sum_t as template type (+ i have defined std types)
// basically i'm passing the types to be used inside the class (not the values!!)
template<uint8_t N, class input_t = uint16_t, class sum_t = uint32_t> 
class Moving_Average
{
public:
/**  One of the primary goal when overloading operator() is to create a functor. A functor acts just like a function, 
 *   but it has the advantages that it is stateful, meaning it can keep data reflecting its state between calls.
 *   (important here for sum and index as they are preserved @ every call)
 *   Otherwise i would have needed a dynamically allocated vector to pass to the function...
 */

    input_t operator() (input_t in_value){ //like: int function(int val){}
        sum -= old_samples[index];  // subtract the oldest sample
        sum += in_value;            // add the newest
        old_samples[index] = input;
        index++;
        if(index == N){
            index = 0;
        }

        return sum/N;
    }

    /** Idea: circular buffer memorize the old values, instead of scanning the buffer @ every call
    *   we keep track of the sum with a proper variable. We use the buffer just to remove the 
    *   oldest value. (Note: first 10 values of the filter are wrong as buffer needs to be full)
    */


private:
    uint8_t index = 0;              // index for the buffer
    input_t old_samples[N] = {};    // circular buffer
    sum_t sum = 0;                  // sum of all the N values
};

#endif