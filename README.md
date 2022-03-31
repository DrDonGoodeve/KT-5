# KT-5
KT-5 Knotmeter head - replacing old (broken) Analog guts with cheap digital guts

On the Scampi-30 I recently bought was installed a non-working Knotmeter - an SR Mariner KT-5 - which has been out of production for over a decade.

This measuring system consists of a transducer on a through-hull fitting consisting of a paddlewheel about 3cm across with magnets on the blade tips and a pick-up coil (soft iron core) in the fitting. This produces AC pulses each time the magnet on the blade passes through the coil. 

The pulse train varies in magnitude and frequency with water speed. The original head instrument used a passive rectifier / low-pass filter to create a current through a moving coil galvanometer which moved the needle. Neat, delicate and unfortunately - broken. Water had got into the head and looks like it had corroded connections on the MCG which were not possible to fix with my soldering skills. My exploration broke what I assume was the bad connection where continuity through the galvanometer was lost. 

However the hole in the boat remains - the right size for this instrument - I like the analog look and do not want to drop $350 for a new setup and new transducer. Being the Engineer I am - and clearly not busy enough - this seems like a fun project to try out on a modern, dirt-cheap microcontroller platform. This project runs on a Raspberry Pi Nano. The project documentation includes all source, pictures and design info. I hope it is useful to someone out there. Also I am documenting the design process through a youtube video - check out Don Goodeve on YouTube.
