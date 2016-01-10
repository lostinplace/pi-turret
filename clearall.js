var Gpio = require('onoff').Gpio, // Constructor function for Gpio objects.
  leds = [];



for (var i =0;i<=27;i++){
  var thisPin = new Gpio(i, 'out');
  leds.push(thisPin);

}

function finish(){
leds.forEach(function(x){
  console.log(JSON.stringify(x));
  try{
    x.writeSync(0);
  }catch(ex){}
  x.unexport();
  console.log(`pin ${x.gpio} done`);
})
};
finish();
