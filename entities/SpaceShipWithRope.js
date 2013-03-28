Ses.Entities.SpaceShipWithRope = Ses.Core.Entity.extend({

   init: function(ship)
   {
      this.ship = ship;
      this.body = ship.body;
      this.linkLength = Ses.Constans.SpaceShip.Rope.Length /
                        Ses.Constans.SpaceShip.Rope.NumberOfLinks;
      this.links = [];
      this.graphics = new createjs.Graphics();

      var link = null;
      var shipPos = this.ship.body.GetWorldCenter();

      var ropeAnchor = this.makeAnchorOnShip();
      var linkWidth = Ses.Constans.SpaceShip.Rope.Width;
      var linkLength = this.linkLength;

      for(var i=0; i<Ses.Constans.SpaceShip.Rope.NumberOfLinks; ++i)
      {
         var position = { x: shipPos.x, y: shipPos.y + 1 + linkLength*i };
         //var newlink = Ses.Physic.createRectangleObject(
         //      linkWidth,
         //      linkLength/2,
         //      position,
         //      { density: Ses.Constans.SpaceShip.Rope.Density }
         //);
         var newlink = Ses.Physic.createCircleObject(
               linkLength/2,
               position,
               { density: Ses.Constans.SpaceShip.Rope.Density }
         );
         if(i === 0)
            Ses.Physic.createRevoluteJoint(
                  ropeAnchor,
                  newlink,
                  new Ses.b2Vec2(0, 0.8),
                  new Ses.b2Vec2(0, -this.linkLength/2)
            );
         else
            Ses.Physic.createRevoluteJoint(
                  link,
                  newlink,
                  new Ses.b2Vec2(0, this.linkLength/2),
                  new Ses.b2Vec2(0, -this.linkLength/2)
            );
         link = newlink;
         this.links.push(link);
      }
      this.grasper = new Ses.Entities.Grasper(link);
      this.shape = new createjs.Container();
      this.shape.addChild(new createjs.Shape(this.graphics));
      this.shape.addChild(this.ship.shape);
      this.shape.addChild(this.grasper.shape);
   },

   update: function(stage)
   {
      this.ship.update(stage);
      this.grasper.update(stage);

      //var begin = this.links[0].GetWorldCenter();
      var begin = this.body.GetWorldCenter();
      this.graphics.clear();
      this.graphics
         .setStrokeStyle(1)
         .beginStroke('#ffffff')
         .moveTo(begin.x*30, begin.y*30);

      for(var i = 0; i < this.links.length; ++i)
      {
         var next = this.links[i].GetWorldCenter();
         this.graphics.lineTo(next.x*30, next.y*30);
      }

      //this.graphics.endStroke();
      
      //this.graphics.rect(begin.x*30, begin.y*30, 20, 20);


      if(stage.AttachKeyDown)
         this.grasper.detach();

   },

   makeAnchorOnShip: function()
   {
      var pos = this.ship.body.GetWorldCenter();
      var shipAnchor = Ses.Physic.createRectangleObject(
         0.05, this.linkLength/2,
         { x: pos.x, y: pos.y }
      );
      Ses.Physic.createRevoluteJoint(
            this.ship.body,
            shipAnchor,
            new Ses.b2Vec2(0, 0),
            new Ses.b2Vec2(0, 0)
      );
      return shipAnchor;
   },

   // delegate
   setOnDieListener: function(callback)
   {
      this.ship.setOnDieListener(callback);
   }
});
