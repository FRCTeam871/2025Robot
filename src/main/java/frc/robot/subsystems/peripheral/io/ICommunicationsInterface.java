package frc.robot.subsystems.peripheral.io;

public interface ICommunicationsInterface {

    void send(IPacket packet);

    void read(int read, IPacket emptyPacket);

    int numAvailable();

    boolean numAvailableSupported();
}
