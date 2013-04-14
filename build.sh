#! /bin/bash

output=build/ses.js

files=(
      "lib/Box2dWeb-2.1.a.3.min.js"
      "lib/yepnope.1.5.3-min.js"
      "lib/easeljs-0.6.0.min.js"
      "lib/tweenjs-0.4.0.min.js"
      "lib/stats.min.js"
      "lib/b2Separator.js"
      "core/Class.js"
      "core/Entity.js"
      "core/EntityPool.js"
      "engine/Constans.js"
      "engine/Engine.js"
      "engine/Physic.js"
      "engine/View.js"
      "engine/GameView.js"
      "engine/Navigator.js"
      "entities/SpaceShip.js"
      "entities/SpaceRock.js"
      "entities/SpaceShipWithDistanceStick.js"
      "entities/Grasper.js"
      "entities/StingGrasper.js"
      "entities/Sensor.js"
      "entities/CircleSensor.js"
      "entities/RectangleSensor.js"
      "entities/JetExhaustParticle.js"
      "entities/BrokenShip.js"
      "entities/TextField.js"
      "engine/Factory.js"
      "ui/ArmourBars.js"
      "ui/Menu.js"
   )


echo "/* Created by Kamil Misiowiec */" > $output
for i in "${files[@]}"
do
   cat $i >> $output
done

cd build
java -jar compiler-latest/compiler.jar --language_in ECMASCRIPT5 --js ses.js --js_output_file ses.min.js
