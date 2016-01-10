var Gpio = require('onoff').Gpio, // Constructor function for Gpio objects.
  leds = [];



for (var i =0;i<=27;i++){
  var thisPin = new Gpio(i, 'out');
  console.log(`pin ${i} on`)
  thisPin.writeSync(1)
  leds.push(thisPin);

}

function finish(){
leds.forEach(function(x){
  x.writeSync(0);
  x.unexport();
  console.log(`pin ${x.gpio} done`);
})
};

setTimeout(finish, 1000);
