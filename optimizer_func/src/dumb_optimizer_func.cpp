#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"


int case_number = 0;

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res)
{
  osrf_gear::Shipment empty_shipment;
  
  if (((req.loaded.products.size() > 0) &&		\
       (req.orphaned.products.size() == 0) &&		\
       (req.missing.products.size() == 0) &&		\
       (req.reposition.products.size() == 0)) ||	\
      (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP)) {

    // Set the decision to ship with label and start loading next shipment
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;

    // What is the next shipment
    switch (case_number) {
    case 0:
      break;
    case 1:
      if (current.shipments.size() > 1) {
	case_number = 2;
	res.shipment = current.shipments[1];
	
      } else {
	case_number = 3;
	res.shipment = priority.shipments[0];
      }
      break;
    case 2:
      case_number = 3;
      res.shipment = priority.shipments[0];
      break;
    case 3:
      if (priority.shipments.size() > 1) {
	case_number = 4;
	res.shipment = priority.shipments[1];
      }
      break;
    case 4:
      case_number = 5;
      res.shipment = empty_shipment;
      break;
    }	
    
  } else {

    // Set the decision to keep loading the current shipment
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;

    // What is the current shipment being loaded
    switch (case_number) {
    case 0:
      if (current.shipments.size() > 0) {
	case_number = 1;
	res.shipment = current.shipments[0];
      } else {
	res.shipment = empty_shipment;
      }
      break;
    case 1:
      res.shipment = current.shipments[0];
      break;
    case 2:
      res.shipment = current.shipments[1];
      break;
    case 3:
      res.shipment = priority.shipments[0];
      break;
    case 4:
      res.shipment = priority.shipments[1];
      break;
    default:
      res.shipment = empty_shipment;
    }
  }
  return true;
}

