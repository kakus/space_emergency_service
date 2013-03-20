/*
 * Pure virtual class !
 */
Ses.Entities.Sensor = Ses.Core.Entity.extend({

   setOnObjectLeaveListener: function(callback)
   {
      if(!this.body)
         throw new Error('Sensor doesn\'t have body');

      Ses.Physic.addOnEndContactListener(this.body, callback);
   },

   setOnObjectEnterListener: function(callback)
   {
      if(!this.body)
         throw new Error('Sensor doesn\'t have body');

      Ses.Physic.addOnBeginContactListener(this.body, callback);
   }

});
