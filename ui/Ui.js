Ses.Ui = Class.extend({

   init: function(stage)
   {
      this.stage = stage;
   },

   setupShipStats: function(ship)
   {
      var text = new createjs.Text("a\na\na", "16px Mono", "#ffffff");
      text.x = 0;
      text.y = this.stage.canvas.height - text.getMeasuredHeight();
      this.stage.addChild(text);

      this.updateShipStats = function() {

         var pos = ship.body.GetWorldCenter(),
             vel = ship.body.GetLinearVelocity();

         text.text =
            "Position [ x: "+pos.x.toFixed(2)+" y: "+pos.y.toFixed(2)+" ]\n"+
            "Velocity [ x: "+vel.x.toFixed(2)+" y: "+vel.x.toFixed(2)+" ]\n"+
            "Armour   [ "+ship.armour+" ]";
      };
   },

   update: function()
   {
      this.updateShipStats();
   }

});
