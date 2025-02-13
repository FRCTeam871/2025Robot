package frc.robot.subsystems.peripheral.io;

public interface IPacket {
    
    byte[] serialize();
    void deserialize(byte[] data);
    
    int getSize();
    
}
