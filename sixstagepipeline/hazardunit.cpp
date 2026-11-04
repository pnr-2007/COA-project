#include "sixstage_hazard_unit.hpp"
#include <cstdio>
using namespace hz6;

static int fails = 0;
#define CHECK(c) do{ if(!(c)){ std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } }while(0)

static StageInfo alu(int rd,int a,int b,uint32_t pc=0){
    StageInfo s; s.valid=true; s.pc=pc; s.nextPc=pc+4; s.rd=rd; s.rs1=a; s.rs2=b;
    s.usesRs1=a>=0; s.usesRs2=b>=0; s.regWrite=rd>=0; return s; }
static StageInfo load(int rd,int base){ StageInfo s=alu(rd,base,kNoReg); s.isLoad=true; return s; }
static StageInfo store(int data,int base){ StageInfo s; s.valid=true; s.rs1=base; s.rs2=data;
    s.usesRs1=s.usesRs2=true; s.isStore=true; return s; }
static StageInfo branch(uint32_t pc,int a,int b,bool taken,uint32_t target,bool pt=false,uint32_t ptgt=0){
    StageInfo s=alu(kNoReg,a,b,pc); s.isBranch=true; s.taken=taken; s.target=target;
    s.predTaken=pt; s.predTarget=ptgt; s.nextPc=taken?target:pc+4; return s; }
static StageInfo jmp(uint32_t pc,uint32_t target,bool pt=false,uint32_t ptgt=0){
    StageInfo s; s.valid=true; s.pc=pc; s.isJump=true; s.taken=true; s.target=target;
    s.predTaken=pt; s.predTarget=ptgt; s.nextPc=target; return s; }
static StageInfo jr(uint32_t pc,uint32_t target,bool pt=false,uint32_t ptgt=0){
    StageInfo s=alu(kNoReg,1,kNoReg,pc); s.isJumpReg=true; s.taken=true; s.target=target;
    s.predTaken=pt; s.predTarget=ptgt; s.nextPc=target; return s; }
static bool allCleared(const HazardOutputs& o){
    return o.ifid.clear && o.idex.clear && o.exmem1.clear && o.mem1mem2.clear && o.mem2wb.clear; }

int main(){
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,5,6); i.mem1=alu(5,1,2);
      auto o=h.evaluate(i); CHECK(o.fwdA==Fwd::FromMEM1 && o.fwdB==Fwd::None); }
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,5,6); i.mem2=alu(6,1,2);
      auto o=h.evaluate(i); CHECK(o.fwdB==Fwd::FromMEM2 && o.fwdA==Fwd::None); }
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,5,6); i.wb=alu(5,1,2);
      auto o=h.evaluate(i); CHECK(o.fwdA==Fwd::FromWB); }
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,5,6); i.mem1=alu(5,1,2); i.wb=alu(5,3,4);
      auto o=h.evaluate(i); CHECK(o.fwdA==Fwd::FromMEM1); }
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,0,0); i.mem1=alu(0,1,2);
      auto o=h.evaluate(i); CHECK(o.fwdA==Fwd::None && o.fwdB==Fwd::None); }
    { HazardUnit6 h; HazardInputs i; i.id=alu(9,5,6); i.wb=alu(6,1,2);
      auto o=h.evaluate(i); CHECK(!o.bypassWBtoID_A && o.bypassWBtoID_B); }
    { HazardUnit6 h; HazardInputs i; i.ex=alu(9,5,6); StageInfo b=alu(5,1,2); b.valid=false; i.mem1=b;
      auto o=h.evaluate(i); CHECK(o.fwdA==Fwd::None); }

    { HazardUnit6 h; HazardInputs i; i.id=alu(9,5,6); i.ex=load(5,1);
      auto o=h.evaluate(i);
      CHECK(!o.pcWrite && !o.ifid.write && o.idex.clear && o.stallReason==StallReason::LoadUse_EX);
      i.ex=StageInfo{}; i.mem1=load(5,1);
      o=h.evaluate(i); CHECK(!o.pcWrite && o.stallReason==StallReason::LoadUse_MEM1);
      i.mem1=StageInfo{}; i.mem2=load(5,1);
      o=h.evaluate(i); CHECK(o.pcWrite && !o.idex.clear);
      HazardInputs j; j.ex=alu(9,5,6); j.wb=load(5,1);
      o=h.evaluate(j); CHECK(o.fwdA==Fwd::FromWB && !o.dataNotReady);
      CHECK(h.stats().loadUseStalls==2); }
    { HazardUnit6 h; HazardInputs i; i.id=store(7,2); i.ex=load(7,1);
      auto o=h.evaluate(i); CHECK(!o.pcWrite); }

    { HazardUnit6 h; HazardInputs i; i.id=alu(1,2,3); i.ex=alu(4,5,6); i.memWait=true;
      auto o=h.evaluate(i);
      CHECK(!o.pcWrite && !o.ifid.write && !o.idex.write && !o.exmem1.write && !o.mem1mem2.write);
      CHECK(o.mem2wb.clear && o.stallReason==StallReason::MemoryWait);
      CHECK(h.stats().memWaitCycles==1); }
    { HazardUnit6 h; HazardInputs i; i.id=alu(1,2,3); i.ex=alu(4,5,6); i.exBusy=true;
      auto o=h.evaluate(i);
      CHECK(!o.pcWrite && !o.ifid.write && !o.idex.write && o.exmem1.clear && o.mem1mem2.write);
      CHECK(o.stallReason==StallReason::ExecBusy); }
    { HazardUnit6 h; HazardInputs i; i.ex=branch(0x100,1,2,true,0x200); i.memWait=true;
      auto o=h.evaluate(i); CHECK(!o.redirect); }

    { BranchPredictor p(PredictorKind::TwoBit,16);
      CHECK(!p.predict(0x40).taken);
      p.update(0x40,true,0x80);
      auto q=p.predict(0x40); CHECK(q.taken && q.btbHit && q.target==0x80);
      p.update(0x40,false,0x80); p.update(0x40,false,0x80);
      CHECK(!p.predict(0x40).taken); }
    { BranchPredictor p(PredictorKind::OneBit,16);
      p.update(0x40,true,0x80); CHECK(p.predict(0x40).taken);
      p.update(0x40,false,0x80); CHECK(!p.predict(0x40).taken); }
    { BranchPredictor p(PredictorKind::StaticNotTaken,16);
      p.update(0x40,true,0x80); CHECK(!p.predict(0x40).taken); }
    { BranchPredictor p(PredictorKind::StaticTaken,16);
      CHECK(!p.predict(0x40).taken);
      p.update(0x40,true,0x80); CHECK(p.predict(0x40).taken); }

    { HazardUnit6 h; HazardInputs i; i.id=alu(9,5,6);
      i.ex=branch(0x100,1,2,true,0x200,true,0x200);
      auto o=h.evaluate(i); CHECK(!o.redirect && !o.ifid.clear && !o.idex.clear);
      CHECK(h.stats().branches==1 && h.stats().mispredicts==0); }
    { HazardUnit6 h; HazardInputs i; i.id=alu(9,5,6);
      i.ex=branch(0x100,1,2,true,0x200,false,0);
      auto o=h.evaluate(i);
      CHECK(o.redirect && o.redirectSource==RedirectSource::EX && o.redirectPc==0x200);
      CHECK(o.ifid.clear && o.idex.clear && !o.exmem1.clear);
      CHECK(h.predict(0x100).taken); }
    { HazardUnit6 h; HazardInputs i;
      i.ex=branch(0x100,1,2,false,0x200,true,0x200);
      auto o=h.evaluate(i); CHECK(o.redirect && o.redirectPc==0x104); }
    { HazardUnit6 h; HazardInputs i;
      i.ex=branch(0x100,1,2,true,0x200,true,0x300);
      auto o=h.evaluate(i); CHECK(o.redirect && o.redirectPc==0x200); }
    { HazardUnit6 h; HazardInputs i; i.ex=branch(0x100,1,2,false,0x200,false,0);
      auto o=h.evaluate(i); CHECK(!o.redirect); }
    { HazardUnit6 h; HazardInputs i; i.ex=jr(0x100,0x500);
      auto o=h.evaluate(i); CHECK(o.redirect && o.redirectPc==0x500 && o.ifid.clear && o.idex.clear); }
    { HazardUnit6 h; HazardInputs i; i.ex=jr(0x100,0x500,true,0x500);
      auto o=h.evaluate(i); CHECK(!o.redirect); }

    { HazardUnit6 h; HazardInputs i; i.id=jmp(0x10,0x400);
      auto o=h.evaluate(i);
      CHECK(o.redirect && o.redirectSource==RedirectSource::ID && o.redirectPc==0x400);
      CHECK(o.ifid.clear && !o.idex.clear); }
    { HazardUnit6 h; HazardInputs i; i.id=jmp(0x10,0x400,true,0x400);
      auto o=h.evaluate(i); CHECK(!o.redirect && !o.ifid.clear); }
    { HazardUnit6 h; HazardInputs i; i.id=jmp(0x10,0x400); i.ex=branch(0x100,1,2,true,0x200);
      auto o=h.evaluate(i); CHECK(o.redirectSource==RedirectSource::EX && o.redirectPc==0x200); }

    { HazardUnit6 h(PredictorKind::TwoBit,64,0x80); HazardInputs i;
      i.wb=alu(5,1,2,0x300); i.wb.exception=ExcCause::IllegalInstr;
      i.mem2=store(1,2); i.memWait=true;
      auto o=h.evaluate(i);
      CHECK(o.trap.taken && !o.trap.isInterrupt && o.trap.epc==0x300);
      CHECK(o.trap.cause==uint32_t(ExcCause::IllegalInstr));
      CHECK(o.redirect && o.redirectPc==0x80 && o.redirectSource==RedirectSource::Trap);
      CHECK(allCleared(o) && o.suppressWBWrite && o.squashMEM2Access && o.pcWrite);
      CHECK(!h.interruptsEnabled() && h.epc()==0x300); }
    { HazardUnit6 h; HazardInputs i;
      i.wb=alu(5,1,2,0x300); i.irqPending=0b1010;
      auto o=h.evaluate(i);
      CHECK(o.trap.taken && o.trap.isInterrupt && o.trap.cause==(0x80000000u|1u));
      CHECK(o.trap.epc==0x304 && !o.suppressWBWrite && allCleared(o)); }
    { HazardUnit6 h; HazardInputs i; h.setIrqMask(0b0100);
      i.wb=alu(5,1,2,0x300); i.irqPending=0b1010;
      auto o=h.evaluate(i); CHECK(!o.trap.taken); }
    { HazardUnit6 h; HazardInputs i; h.setInterruptsEnabled(false);
      i.wb=alu(5,1,2,0x300); i.irqPending=1;
      auto o=h.evaluate(i); CHECK(!o.trap.taken); }
    { HazardUnit6 h; HazardInputs i; i.irqPending=1;
      auto o=h.evaluate(i); CHECK(!o.trap.taken);
      i.wb=alu(5,1,2,0x300);
      o=h.evaluate(i); CHECK(o.trap.taken && o.trap.cause==0x80000000u); }
    { HazardUnit6 h; HazardInputs i;
      i.wb=alu(5,1,2,0x300); i.irqPending=1;
      auto o=h.evaluate(i); CHECK(o.trap.taken);
      auto o2=h.evaluate(i); CHECK(!o2.trap.taken);
      HazardInputs r; r.wb=alu(kNoReg,kNoReg,kNoReg,0x80); r.wb.isTrapReturn=true;
      auto o3=h.evaluate(r);
      CHECK(o3.trap.taken && o3.trap.isReturn && o3.redirectPc==0x304 && allCleared(o3));
      CHECK(h.interruptsEnabled()); }
    { HazardUnit6 h; HazardInputs i;
      i.wb=alu(5,1,2,0x300); i.wb.exception=ExcCause::EnvCall; i.irqPending=1;
      auto o=h.evaluate(i);
      CHECK(!o.trap.isInterrupt && o.trap.cause==uint32_t(ExcCause::EnvCall)); }
    { HazardUnit6 h; HazardInputs i;
      i.wb=alu(5,1,2,0x300); i.wb.exception=ExcCause::LoadAccessFault;
      i.ex=branch(0x100,1,2,true,0x200);
      auto o=h.evaluate(i); CHECK(o.redirectPc==0x80 && o.trap.taken); }

    std::printf(fails ? "%d test(s) FAILED\n" : "All tests passed\n", fails);
    return fails != 0;
}
