function f0_oneFoot = processOneFootOutputQP(primalSolution,LR_FootInContact)

r_inContact = LR_FootInContact(1);
l_inContact = LR_FootInContact(2);
f0_oneFoot  = [primalSolution * r_inContact ; primalSolution * l_inContact]*abs(r_inContact - l_inContact);

end