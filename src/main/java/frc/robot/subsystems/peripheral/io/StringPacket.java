package frc.robot.subsystems.peripheral.io;

public class StringPacket implements IPacket{

    String payload;

    public StringPacket() {}

    public StringPacket(String payload) {
        this.payload = payload;
    }

    @Override
    public byte[] serialize() {
        return payload.getBytes();
    }

    @Override
    public void deserialize(byte[] data) {
        payload = new String(data);
    }

    @Override
    public int getSize() {
        return payload == null ? 0 : payload.length();
    }

    public void setPayload(String payload){
        this.payload = payload;
    }

    public String getPayload(){
        return payload;
    }

}
