Ses.CssUi = (function(){

   var m_ship,
       game_div = document.getElementById('game_div'),
       shipStats = document.createElement('div');

   // ShipStats construction
   shipStats.id = 'ship_stats';
   shipStats.innerHTML = 'ShipStats';
   shipStats.className = 'ui';
   shipStats.style.display = 'none';

   game_div.appendChild(shipStats);


   var updateShipStats = function()
   {
      if (!m_ship)
         return;

      var pos = m_ship.body.GetWorldCenter(),
          vel = m_ship.body.GetLinearVelocity();

      shipStats.innerHTML =  
         "<table cellpadding='0' cellspacing='3'>"
         + "	<tr><td></td><td>x</td><td>y</td></tr>"
         + "	<tr>"
         + "		<td>Position</td>"
         + "		<td>"+pos.x.toFixed(2)+"</td>"
         + "		<td>"+pos.y.toFixed(2)+"</td>"
         + "	</tr>"
         + "	<tr>"
         + "		<td>Velocity</td>"
         + "		<td>"+vel.x.toFixed(2)+"</td>"
         + "		<td>"+vel.y.toFixed(2)+"</td>"
         + "	</tr>"
         + "	<tr>"
         + "		<td>Armour</td>"
         + "		<td>"+m_ship.armour.toFixed(1)+"</td>"
         + "		<td></td>"
         + "	</tr>"
         + "</table>";

   };

   return {

      update: function() {
         updateShipStats();
      },

      initShipStats: function(ship) {
         m_ship = ship;
         shipStats.style.display = 'block';
      },

      hideSipStats: function() {
         shipStats.style.display = 'none';
      }

   };

})();
