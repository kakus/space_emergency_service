Ses.Engine.Factory = {

   gameView: null,

   factoryMethod: {

      'SpaceRock': function(des) {
         if(des.radious)
            return new Ses.Entities.SpaceRock(des.x, des.y, des.radious);

         Ses.log("Create space rock from width");
         return new Ses.Entities.SpaceRock(des.x, des.y, 2);
      },

      'SpaceShip': function(des) {
         return new Ses.Entities.SpaceShip(des.x, des.y);
      },

      'SpaceShipWithRope': function(des) {
         return new Ses.Entities.SpaceShipWithRope(this.SpaceShip(des));
      },

      'CircleSensor': function(des) {
         return new Ses.Entities.CircleSensor(des.x, des.y, des.width);
      },

      'RectangleSensor': function(des) {
         return new Ses.Entities.RectangleSensor(
               des.x, des.y, des.width/2, des.height/2);
      }
   },

   /* Creates object by the given description, also adds objectives to the
    * current map so we can know when game should end.
    */
   createObject: function(description)
   {
      var obj = this.factoryMethod[description.type](description);

      var isObjective = /O_.*/;
      for (var field in description)
         if (isObjective.test(field))
            this.gameView.addMapObjective(field, obj);

      return obj;
   }
};
