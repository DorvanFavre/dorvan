import 'package:dorvan/application/constants/personal_infos.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/presentation/white_button.dart';
import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:url_launcher/url_launcher.dart';

import 'aware_text.dart';
import 'second_scroll_view.dart';

class ContactView extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Expanded(
            child: Padding(
          padding: const EdgeInsets.all(30.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Spacer(
                flex: 3,
              ),
              // Contact
              AwareText(
                'contact_title',
                builder: (text) => Text(
                  text,
                  style: TextStyle(color: Colors.white, fontSize: 30),
                ),
              ),
              SizedBox(height: 20),
              AwareText(
                'contact_description',
                builder: (text) => Text(
                  text,
                  style: TextStyle(color: Colors.white60, fontSize: 14),
                ),
              ),
              SizedBox(height: 30),
              Row(
                children: [
                  IconButton(
                    icon: FaIcon(
                      FontAwesomeIcons.instagram,
                      color: Colors.white,
                    ),
                    onPressed: () {
                      _launchInstagram(context);
                    },
                  ),
                  IconButton(
                    icon:
                        FaIcon(FontAwesomeIcons.whatsapp, color: Colors.white),
                    onPressed: () {
                      _launchWhatsapp(context);
                    },
                  ),
                  IconButton(
                    icon:
                        FaIcon(FontAwesomeIcons.envelope, color: Colors.white),
                    onPressed: () {
                      _launchMailClient(context);
                    },
                  )
                ],
              ),
              Spacer(
                flex: 1,
              ),
              Divider(
                color: Colors.grey,
              ),
              Spacer(
                flex: 1,
              ),
              // Tarifs
              /*AwareText(
                'tarif_title',
                builder: (text) => Text(
                  text,
                  style: TextStyle(color: Colors.white, fontSize: 30),
                ),
              ),
              SizedBox(height: 20),
              AwareText(
                'tarif_description',
                builder: (text) => Text(
                  text,
                  style: TextStyle(color: Colors.white60, fontSize: 14),
                ),
              ),
              SizedBox(height: 30),
              //WhiteButton(onTap: () {}, text: 'see more'),
              Spacer(
                flex: 2,
              ),*/
            ],
          ),
        )),
        Container(
          height: 150,
          color: Colors.grey[850],
          child: Padding(
            padding: EdgeInsets.all(30),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    infoText('Dorvan Favre'),
                    infoText('079 571 92 69'),
                    infoText('dorvan.favre@gmail.com'),
                  ],
                ),
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    infoText('Au petit-chêne 19'),
                    infoText('2502 Bienne'),
                    infoText('Suisse'),
                  ],
                )
              ],
            ),
          ),
        )
      ],
    );
  }

  Widget infoText(String text) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Text(
        text,
        style: TextStyle(color: Colors.grey, fontSize: 11),
      ),
    );
  }

  void _launchWhatsapp(BuildContext context) async {
    //var whatsappUrl = "whatsapp://send?phone=$kPhoneNumber";
    final whatsappUrl = "https://api.whatsapp.com/send/?phone=$kWhatsappPhoneNumber";
    try {
      launch(whatsappUrl);
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text("Can't launch Whatsapp"),
      ));
    }
  }

  void _launchInstagram(BuildContext context) {
    final instagramUrl = "https://www.instagram.com/$kInstagram/";
    try {
      launch(instagramUrl);
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text("Can't launch Instagram"),
      ));
    }
  }

  void _launchMailClient(BuildContext context) async {
    final mailUrl = 'mailto:$kEmail';
    try {
      await launch(mailUrl);
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text("Can't launch Email"),
      ));
    }
  }
}
