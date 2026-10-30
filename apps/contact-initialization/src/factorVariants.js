export const factorVariants = [
  {id:'none', title:'Contacts only', between:false, upright:false, ground:false},
  {id:'between', title:'Weak motion', between:true, upright:false, ground:false},
  {id:'upright', title:'Upright', between:false, upright:true, ground:false},
  {id:'both', title:'Motion + upright', between:true, upright:true, ground:false},
  {id:'ground', title:'Landmark height', between:false, upright:false, ground:true},
  {id:'between-ground', title:'Motion + height', between:true, upright:false, ground:true},
  {id:'upright-ground', title:'Upright + height', between:false, upright:true, ground:true},
  {id:'all', title:'All three priors', between:true, upright:true, ground:true},
];

// Automatic selection preserves the user's height-prior choice. Only an
// explicit checkbox or table-row selection may change that assumption.
export function bestTranslationRow(rows, ground) {
  return rows.filter(row=>row.ground===ground).reduce((best,row)=>
    !best||row.run.evaluation.positionRmseMeters<best.run.evaluation.positionRmseMeters?row:best,null);
}
