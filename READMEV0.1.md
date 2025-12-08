# Final-Prj-of-Advanced-computer-architecture
25fall Advanced computer architecture, Final PRJ
By Trevor, Thierry, Yang

we build a cpu in RiscV with special instruction set.

Link of Document
https://docs.google.com/document/d/1RRewOy-fuHN_yRXx5kFoFxv4HNzzx_Hq/edit


/////////////////  bug 1  ///////////////
Simulation successfully ran for 3500ns, but the Data Memory dump shows all zeros,
indicating the Bubble Sort program did not correctly write any data to memory. 
The sorting logic is sound, but the data path for store (sw) instructions 
is being corrupted by a crucial, but simple, data width mismatch warning in top module.
/////////////////////////////////////////
