/* Use this code to check the function of your motors when they are powered on. Proper function will result in each joint moving back and fourth around its initial position. 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
*/
 
#include <Encoder.h>

const int
// motor connected to M1
PWM_1   = 9,
DIR_1   = 7;
const int
// motor connected to M2
PWM_2   = 10,
DIR_2   = 8;

int _nD2 = 4;
int _nSF = 12;

//Define Encoder Pins. First pin must be an interupt pin (see, https://www.pjrc.com/teensy/td_libs_Encoder.html).
  Encoder Mot1(2,6);
  Encoder Mot2(3,5);

//Parameters
  bool direction=HIGH;
  int value=50;  // PWM Duty Cycle, 0 (always off) and 255 (always on)
  int guard=100; // Soft-stop encoder count
  
void setup(){
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH);
  Serial.begin(115200);
  Serial.println("start");
}

void loop(){  
  if (Serial.available()){
    char val = Serial.read();
    Serial.print(val);
    if(val == '1')
    {
      digitalWrite(DIR_1, HIGH);
      analogWrite(PWM_1, (int) constrain(value, 0, 255));  
    }
    else if(val == '2')
    {
      digitalWrite(DIR_1, LOW);
      analogWrite(PWM_1, (int) constrain(value, 0, 255));  
    }
    else if(val == '3')
    {
      digitalWrite(DIR_2, HIGH);
      analogWrite(PWM_2, (int) constrain(value, 0, 255));  
    }
    else if(val == '4')
    {
      digitalWrite(DIR_2, LOW);
      analogWrite(PWM_2, (int) constrain(value, 0, 255));  
    }
    else
    {
      analogWrite(PWM_1, 0);
      analogWrite(PWM_2, 0);
    }
  }
  else
  {
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
  }
  // Read Encoders
  Serial.print("Mot 1: "); //Read encoder
  Serial.print(Mot1.read()); //Read encoder
  Serial.print(" | Mot 2: "); //Read encoder
  Serial.println(Mot2.read()); //Read encoder
  
  delay(10);
 
}
