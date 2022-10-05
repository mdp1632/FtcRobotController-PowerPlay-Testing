
package org.firstinspires.ftc.teamcode;

public class Toggle{
    private boolean InputBool;
    private boolean outputState = false;

    private double lastTime = 0;
    private double lastTime2 = 0;

    private double debounceTime = 250;

    //Parameterized Constructor
    Toggle(boolean InputBool){
        InputBool = this.InputBool;
    }



    public boolean toggleBoolean() {
        if (InputBool) {
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
        }
        return outputState;
    }

    public boolean toggleBooleanDebounced(){
        double currentTime = System.currentTimeMillis();

        if(InputBool && (currentTime > (lastTime + debounceTime))){
            outputState = !outputState;
            lastTime = currentTime;
        }
        return outputState;
    }

    public boolean toggleBooleanDebounced2(boolean InputBool) {
        double currentTime = System.currentTimeMillis();

        if (InputBool && (currentTime > (lastTime2 + debounceTime))){
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
            lastTime2 = currentTime;
        }
        return outputState;
    }


}


// TODO: 11/29/2020 add timer for debouncing.
/*
public class Toggle {
    private boolean outputState = false;
    // private double lastTime = 0;
    // private double debounceTime = 250;      //in milliseconds

    public boolean toggleBoolean(boolean InputBool){

        double currentTime = System.currentTimeMillis();
        if (InputBool && (currentTime > (lastTime + debounceTime))){
        if (InputBool){
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
        }
        lastTime = System.currentTimeMillis();
        return outputState;

    }

}
*/